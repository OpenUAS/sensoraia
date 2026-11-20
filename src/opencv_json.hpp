/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file opencv_json.hpp
 * @brief Header-only C++17 value adapter around OpenCV's JSON persistence layer.
 * @details OpenCV remains the parser and ordinary writer. Normalization carries
 * null, booleans and 64-bit integers through reserved string markers so those
 * types are not lost in FileStorage's representation. Nodes are then detached
 * from FileStorage into a heap-backed tree usable by existing C++ consumers.
 *
 * @par Ownership and mutation
 * Ordinary Value copies recursively clone their trees. Child lookups and items()
 * initially return shared child handles instead, allowing a selected child's
 * mutators to affect its parent. Copying such a handle creates an independent
 * tree; assigning a new Value to the handle rebinds the handle, not the parent slot.
 *
 * @par Deliberate limits of this adapter
 * This is not JSONbourne's bounded arena API: strings, maps, vectors and shared
 * nodes allocate, while cloning, conversion and serialization recurse. No fixed
 * nesting or allocation budget is imposed here. Objects use unique sorted map
 * keys rather than retaining duplicate members or their original source order.
 * @pre Application string/key data must not use the reserved
 * __USHER_OPENCV_JSON_ namespace, which belongs to the transport representation.
 * @note OpenCV acceptance and this adapter's normalization determine accepted
 * input; this wrapper is not an independent strict RFC 8259 validator.
 * @see usher_cv_json::Value::parse(), usher_cv_json::Value::dump_sorted()
 */
#ifndef USHER_OPENCV_JSON_HPP
#define USHER_OPENCV_JSON_HPP

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

/** @brief OpenCV-backed JSON values and the internal tree supporting them. */
namespace usher_cv_json {

/**
 * @brief Heap-backed tagged storage independent of a FileStorage lifetime.
 * @details Separate signed, unsigned and real fields avoid forcing 64-bit
 * integers through a double. Only the field selected by kind is semantically
 * active; mutators need not clear storage belonging to previously selected kinds.
 */
struct Node {
  /** @brief Active payload category; boolean is distinct from integer storage. */
  enum class Kind {
    Null,     /**< JSON null with no active scalar payload. */
    Bool,     /**< Native boolean. */
    Signed,   /**< Signed 64-bit integer. */
    Unsigned, /**< Unsigned 64-bit integer. */
    Real,     /**< Double-precision real value. */
    String,   /**< Owned byte string. */
    Array,    /**< Ordered child-node sequence. */
    Object    /**< Unique members in std::map key order. */
  };
  Kind kind = Kind::Null;       /**< Tag governing interpretation and serialization. */
  bool boolean = false;        /**< Bool payload. */
  int64_t signed_number = 0;    /**< Signed payload. */
  uint64_t unsigned_number = 0; /**< Unsigned payload. */
  double real_number = 0;       /**< Real payload; construction alone does not reject nonfinite values. */
  std::string string;           /**< String payload with std::string byte-length semantics. */
  std::vector<std::shared_ptr<Node>> array; /**< Array children in insertion order. */
  std::map<std::string, std::shared_ptr<Node>> object; /**< Unique keys with shared child ownership. */
};

/**
 * @brief C++ value interface with explicit conversions and OpenCV JSON I/O.
 * @details Copy construction/assignment are deep; lookup-produced handles share
 * a selected node until copied or rebound. Conversion failures throw rather than
 * silently turning missing, differently typed or out-of-range data into defaults.
 * @note Allocation failures may additionally propagate standard-library exceptions.
 */
class Value {
public:
  /** @brief Adapter errors exposed independently of OpenCV's exception type. */
  class exception : public std::runtime_error {
  public:
    /** @brief Reuse runtime_error constructors to retain descriptive error text. */
    using std::runtime_error::runtime_error;
  };
  /** @brief One named value supplied to an object factory. */
  using Field = std::pair<std::string, Value>;

  /** @brief Allocate a fresh null node instead of using a shared mutable singleton. */
  Value() : node_(std::make_shared<Node>()) {}
  /** @brief Construct null from the C++ nullptr tag. */
  Value(std::nullptr_t) : Value() {}
  /**
   * @brief Construct a boolean without confusing it with an integral payload.
   * @param[in] value Boolean to store.
   */
  Value(bool value) : Value() {
    node_->kind = Node::Kind::Bool;
    node_->boolean = value;
  }
  /**
   * @brief Copy a terminated string; a null pointer selects an empty string.
   * @param[in] value C string, or nullptr; embedded NULs require the std::string overload.
   */
  Value(const char *value) : Value(std::string(value ? value : "")) {}
  /**
   * @brief Own a complete byte string rather than borrowing external storage.
   * @param[in] value String copied with its explicit byte length.
   */
  Value(const std::string &value) : Value() {
    node_->kind = Node::Kind::String;
    node_->string = value;
  }

  /**
   * @brief Store native integral input in its signed or unsigned 64-bit category.
   * @tparam Number Integral source type other than bool.
   * @param[in] value Number converted to the corresponding 64-bit storage type.
   * @note bool is excluded so overload resolution preserves boolean semantics.
   */
  template <typename Number,
            typename = std::enable_if_t<std::is_integral_v<Number> &&
                                        !std::is_same_v<Number, bool>>>
  Value(Number value) : Value() {
    if constexpr (std::is_signed_v<Number>) {
      node_->kind = Node::Kind::Signed;
      node_->signed_number = static_cast<int64_t>(value);
    } else {
      node_->kind = Node::Kind::Unsigned;
      node_->unsigned_number = static_cast<uint64_t>(value);
    }
  }
  /**
   * @brief Store floating-point input as a double, not an integer-shaped value.
   * @tparam Number Floating-point source type.
   * @param[in] value Number converted to double; this constructor does not check finiteness.
   */
  template <typename Number,
            typename = std::enable_if_t<std::is_floating_point_v<Number>>,
            typename = void>
  Value(Number value) : Value() {
    node_->kind = Node::Kind::Real;
    node_->real_number = static_cast<double>(value);
  }

  /**
   * @brief Recursively detach a copy so later mutation does not alias the source.
   * @param[in] other Accessible source value, not a moved-from handle.
   */
  Value(const Value &other) : node_(clone(other.node_)) {}
  /** @brief Transfer node ownership; only destruction/reassignment is assumed for the source. */
  Value(Value &&) noexcept = default;
  /**
   * @brief Deep-copy and rebind this handle without modifying any former parent slot.
   * @param[in] other Source value; self-assignment is permitted.
   * @return This handle after assignment.
   */
  Value &operator=(const Value &other) {
    if (this != &other)
      node_ = clone(other.node_);
    return *this;
  }
  /**
   * @brief Move/rebind ownership without recursively cloning nodes.
   * @return This handle; the moved-from source must be reassigned before inspection.
   */
  Value &operator=(Value &&) noexcept = default;

  /** @brief Make an empty object. @return Independently owned object value. */
  static Value object() {
    Value value;
    value.node_->kind = Node::Kind::Object;
    return value;
  }
  /**
   * @brief Build an object by cloning each supplied member value.
   * @param[in] fields Named values; later repeated keys replace earlier members.
   * @return Object in map key order rather than initializer/source order.
   */
  static Value object(std::initializer_list<Field> fields) {
    Value value = object();
    for (const auto &field : fields)
      value.set(field.first, field.second);
    return value;
  }
  /** @brief Make an empty array. @return Independently owned array value. */
  static Value array() {
    Value value;
    value.node_->kind = Node::Kind::Array;
    return value;
  }
  /**
   * @brief Build an ordered array by cloning each supplied value.
   * @param[in] values Elements in their desired order.
   * @return Independently owned array value.
   */
  static Value array(std::initializer_list<Value> values) {
    Value value = array();
    for (const auto &item : values)
      value.push_back(item);
    return value;
  }

  /**
   * @brief Copy a byte range and delegate parsing to the string overload.
   * @tparam Iterator Iterator type accepted by std::string's range constructor.
   * @param[in] first Beginning of the input range.
   * @param[in] last One-past-end iterator.
   * @return Tree detached from OpenCV's temporary storage.
   * @throws exception If normalization or OpenCV input processing fails.
   */
  template <typename Iterator>
  static Value parse(Iterator first, Iterator last) {
    return parse(std::string(first, last));
  }
  /**
   * @brief Parse terminated input; use a range/std::string for an explicit extent.
   * @param[in] input Terminated bytes; nullptr selects an empty input string.
   * @return Parsed tree, independent of the input buffer.
   * @throws exception If the selected input cannot be read.
   */
  static Value parse(const char *input) {
    return parse(std::string(input ? input : ""));
  }
  /**
   * @brief Normalize compatibility tokens, parse with OpenCV and detach the tree.
   * @param[in] input JSON input whose original bytes remain unchanged.
   * @return Independently owned tree with restored bool/null/integer categories.
   * @throws exception For rejected sentinels/escapes, conversion or OpenCV parse errors.
   * @note cv::Exception is translated at this boundary so callers can use the
   * adapter's error type without depending on OpenCV's parser diagnostics API.
   */
  static Value parse(const std::string &input) {
    const std::string normalized = normalize_for_opencv(input);
    try {
      cv::FileStorage storage(normalized, cv::FileStorage::READ |
                                              cv::FileStorage::MEMORY |
                                              cv::FileStorage::FORMAT_JSON);
      if (!storage.isOpened())
        throw exception("OpenCV FileStorage could not open JSON input");
      return from_node(storage.root());
    } catch (const cv::Exception &error) {
      throw exception(std::string("invalid OpenCV JSON: ") + error.what());
    }
  }

  /** @brief Inspect the null tag. @return True only for a null node. */
  bool is_null() const { return node_->kind == Node::Kind::Null; }
  /** @brief Inspect the object tag. @return True only for an object node. */
  bool is_object() const { return node_->kind == Node::Kind::Object; }
  /** @brief Inspect the array tag. @return True only for an array node. */
  bool is_array() const { return node_->kind == Node::Kind::Array; }
  /** @brief Inspect the string tag. @return True only for a string node. */
  bool is_string() const { return node_->kind == Node::Kind::String; }
  /** @brief Test all three numeric categories without treating bool as numeric. @return Numeric tag status. */
  bool is_number() const {
    return node_->kind == Node::Kind::Signed ||
           node_->kind == Node::Kind::Unsigned ||
           node_->kind == Node::Kind::Real;
  }
  /** @brief Test signed/unsigned storage without accepting a real such as 1.0. @return Integer tag status. */
  bool is_number_integer() const {
    return node_->kind == Node::Kind::Signed ||
           node_->kind == Node::Kind::Unsigned;
  }
  /** @brief Inspect unsigned storage rather than testing a number's sign. @return Unsigned tag status. */
  bool is_number_unsigned() const {
    return node_->kind == Node::Kind::Unsigned;
  }
  /**
   * @brief Check member presence without interpreting a null value as missing.
   * @param[in] key Non-null terminated member name.
   * @return True for an existing object member; false for nonobjects/missing keys.
   */
  bool contains(const char *key) const {
    return is_object() && node_->object.find(key) != node_->object.end();
  }
  /**
   * @brief Check a member name through the C-string lookup interface.
   * @param[in] key Name; an embedded NUL ends the portion used for lookup.
   * @return Whether the object contains the selected name.
   */
  bool contains(const std::string &key) const { return contains(key.c_str()); }
  /**
   * @brief Count container entries, not string bytes or scalar payload size.
   * @return Array length, object member count, or zero for a scalar/null.
   */
  size_t size() const {
    return is_array()    ? node_->array.size()
           : is_object() ? node_->object.size()
                         : 0;
  }

  /**
   * @brief Look up a shared child handle, using null for a missing member.
   * @param[in] key Non-null terminated name.
   * @return Handle initially sharing the child, or a fresh null value if absent.
   * @throws exception If this value is not an object; lookup does not insert.
   */
  Value operator[](const char *key) const { return lookup(key, false); }
  /**
   * @brief Perform optional lookup through the terminated-name overload.
   * @param[in] key Name; lookup stops at its first embedded NUL.
   * @return Shared child handle or a fresh null for a missing member.
   * @throws exception If this value is not an object.
   */
  Value operator[](const std::string &key) const {
    return lookup(key.c_str(), false);
  }
  /**
   * @brief Require an object member instead of substituting null.
   * @param[in] key Non-null terminated member name.
   * @return Handle initially sharing the selected child node.
   * @throws exception For a nonobject or missing key.
   */
  Value at(const char *key) const { return lookup(key, true); }
  /**
   * @brief Require a member through the terminated-name interface.
   * @param[in] key Name; an embedded NUL terminates lookup.
   * @return Shared child handle.
   * @throws exception For a nonobject or missing key.
   */
  Value at(const std::string &key) const { return at(key.c_str()); }
  /**
   * @brief Check signed array indices before conversion to an unsigned extent.
   * @param[in] index Zero-based array position.
   * @return Shared child handle.
   * @throws exception For a negative/out-of-range index or nonarray value.
   */
  Value operator[](int index) const { return at(index); }
  /**
   * @brief Select an array element with the same bounds policy as at().
   * @param[in] index Zero-based array position.
   * @return Shared child handle.
   * @throws exception For a nonarray or out-of-range position.
   */
  Value operator[](size_t index) const { return at(index); }
  /**
   * @brief Reject a negative position before any size_t cast can wrap it.
   * @param[in] index Signed zero-based array position.
   * @return Shared child handle.
   * @throws exception For invalid array access.
   */
  Value at(int index) const {
    if (index < 0)
      throw exception("JSON array index out of range");
    return at(static_cast<size_t>(index));
  }
  /**
   * @brief Check both the array kind and its actual vector extent.
   * @param[in] index Zero-based array position.
   * @return Handle sharing the selected node, not a recursive copy.
   * @throws exception For a nonarray or out-of-range position.
   */
  Value at(size_t index) const {
    if (!is_array() || index >= node_->array.size())
      throw exception("JSON array index out of range");
    return Value(node_->array[index]);
  }

  /**
   * @brief Convert explicitly while preserving the distinction between types.
   * @tparam Type Value, string, bool, arithmetic type, std::array or std::vector;
   * container elements are converted recursively using their value_type.
   * @return Converted value; requesting Value makes a deep copy.
   * @throws exception For incompatible tags, nonfinite/out-of-range numeric
   * conversion or a fixed-size array whose length does not match.
   * @note Integer targets reject real-tagged inputs even if mathematically
   * integral. Floating conversions use double and do not promise integer precision.
   * Unsupported target types produce a compile-time error, not a runtime fallback.
   */
  template <typename Type> Type get() const {
    if constexpr (std::is_same_v<Type, Value>)
      return *this;
    else if constexpr (std::is_same_v<Type, std::string>) {
      if (!is_string())
        throw exception("JSON value is not a string");
      return node_->string;
    } else if constexpr (std::is_same_v<Type, bool>) {
      if (node_->kind != Node::Kind::Bool)
        throw exception("JSON value is not a boolean");
      return node_->boolean;
    } else if constexpr (std::is_floating_point_v<Type>) {
      if (!is_number())
        throw exception("JSON value is not numeric");
      const double number = node_->kind == Node::Kind::Real ? node_->real_number
                            : node_->kind == Node::Kind::Signed
                                ? static_cast<double>(node_->signed_number)
                                : static_cast<double>(node_->unsigned_number);
      if (!std::isfinite(number) ||
          number < -std::numeric_limits<Type>::max() ||
          number > std::numeric_limits<Type>::max())
        throw exception("JSON number is out of range");
      return static_cast<Type>(number);
    } else if constexpr (std::is_integral_v<Type> && std::is_signed_v<Type>) {
      if (!is_number_integer())
        throw exception("JSON value is not an integer");
      const long double number =
          node_->kind == Node::Kind::Signed
              ? static_cast<long double>(node_->signed_number)
              : static_cast<long double>(node_->unsigned_number);
      if (number < std::numeric_limits<Type>::min() ||
          number > std::numeric_limits<Type>::max())
        throw exception("JSON integer is out of range");
      return static_cast<Type>(number);
    } else if constexpr (std::is_integral_v<Type> && std::is_unsigned_v<Type>) {
      if (!is_number_integer())
        throw exception("JSON value is not an integer");
      const long double number =
          node_->kind == Node::Kind::Signed
              ? static_cast<long double>(node_->signed_number)
              : static_cast<long double>(node_->unsigned_number);
      if (number < 0 || number > std::numeric_limits<Type>::max())
        throw exception("JSON integer is out of range");
      return static_cast<Type>(number);
    } else if constexpr (is_std_array<Type>::value) {
      if (!is_array() || size() != std::tuple_size_v<Type>)
        throw exception("JSON array has unexpected length");
      Type result{};
      for (size_t index = 0; index < result.size(); ++index)
        result[index] = at(index).template get<typename Type::value_type>();
      return result;
    } else if constexpr (is_std_vector<Type>::value) {
      if (!is_array())
        throw exception("JSON value is not an array");
      Type result;
      result.reserve(size());
      for (size_t index = 0; index < size(); ++index)
        result.push_back(at(index).template get<typename Type::value_type>());
      return result;
    } else
      static_assert(!sizeof(Type), "unsupported JSON conversion type");
  }

  /**
   * @brief Use a default only for absence, never to hide an invalid present value.
   * @tparam Type Conversion/default type supported by get().
   * @param[in] key Non-null terminated member name.
   * @param[in] fallback Result when no member exists (including on a nonobject).
   * @return Converted member or fallback.
   * @throws exception If a present member cannot be converted to Type.
   */
  template <typename Type> Type value(const char *key, Type fallback) const {
    return contains(key) ? at(key).template get<Type>() : std::move(fallback);
  }
  /**
   * @brief Provide a string-valued default rather than returning a borrowed pointer.
   * @param[in] key Non-null terminated member name.
   * @param[in] fallback Non-null terminated default string.
   * @return Owned member/default string.
   * @throws exception If a present member is not a string.
   */
  std::string value(const char *key, const char *fallback) const {
    return contains(key) ? at(key).get<std::string>() : std::string(fallback);
  }

  /**
   * @brief Snapshot object names and shared child handles in map order.
   * @return Newly allocated member vector; child nodes initially remain shared.
   * @throws exception If this value is not an object.
   * @note The vector is detached from later map insertion/removal, but a returned
   * child's in-place mutation can still affect the source node it shares.
   */
  std::vector<std::pair<std::string, Value>> items() const {
    if (!is_object())
      throw exception("JSON value is not an object");
    std::vector<std::pair<std::string, Value>> result;
    result.reserve(node_->object.size());
    for (const auto &item : node_->object)
      result.emplace_back(item.first, Value(item.second));
    return result;
  }

  /**
   * @brief Clone a member value, replace any matching key and select object kind.
   * @param[in] key Owned member name copied into the map.
   * @param[in] value Source value cloned rather than linked into this tree.
   * @throws exception If a directly supplied string uses a reserved bridge marker.
   * @note Existing other-kind payload storage is not cleared by this operation.
   */
  void set(const std::string &key, const Value &value) {
    require_unreserved(value);
    node_->object[key] = clone(value.node_);
    node_->kind = Node::Kind::Object;
  }
  /**
   * @brief Construct a supported native value before applying member insertion policy.
   * @tparam Type Type accepted by a Value constructor.
   * @param[in] key Member name.
   * @param[in] value Native value to convert and clone.
   * @throws exception Under the same reserved-string policy as set().
   */
  template <typename Type> void set(const std::string &key, Type value) {
    set(key, Value(value));
  }
  /**
   * @brief Clone an element, append it in order and select array kind.
   * @param[in] value Source element, whose later mutation must not alter this copy.
   * @throws exception If a directly supplied string uses the reserved namespace.
   * @note Other-kind payload storage is retained but no longer selected by kind.
   */
  void push_back(const Value &value) {
    require_unreserved(value);
    node_->array.push_back(clone(value.node_));
    node_->kind = Node::Kind::Array;
  }
  /**
   * @brief Construct a native value before applying array insertion policy.
   * @tparam Type Type accepted by a Value constructor.
   * @param[in] value Native element to convert and clone.
   * @throws exception Under the same reserved-string policy as push_back().
   */
  template <typename Type> void push_back(Type value) {
    push_back(Value(value));
  }
  /**
   * @brief Shallow-merge object members while deep-copying each replacement value.
   * @param[in] other Object whose keys replace matching keys in this value.
   * @throws exception If other is not an object or an inserted string is reserved.
   * @note This merges one member level, not nested objects recursively.
   */
  void update(const Value &other) {
    for (const auto &item : other.items())
      set(item.first, item.second);
  }

  /**
   * @brief Serialize with OpenCV and restore native JSON scalar tokens.
   * @param[in] indent Negative selects compact output; nonnegative keeps OpenCV
   * formatting. Its numeric magnitude is not an indentation-width setting.
   * @return Owned serialized text.
   * @throws exception For adapter-detected writer or reserved-string failures.
   * @note The unnamed character/bool arguments are ignored compatibility slots.
   * OpenCV writer exceptions may propagate as cv::Exception rather than exception.
   */
  std::string dump(int indent = -1, char = ' ', bool = false) const {
    return write_opencv(*this, indent);
  }
  /**
   * @brief Produce compact map-ordered output without FileStorage formatting.
   * @return Owned text with non-ASCII strings expressed as Unicode escapes.
   * @throws exception For malformed/truncated UTF-8 sequence shapes or failed number formatting.
   * @pre Use finite real values and valid Unicode strings when strict JSON is needed.
   * @note This is deterministic map-key ordering, not RFC 8785 canonicalization.
   * UTF-8 checks cover lead/continuation shape and truncation, not every scalar
   * validity rule; real-tagged integral spellings keep a .0 suffix for round trips.
   */
  std::string dump_sorted() const {
    std::string output;
    const auto append_string = [&output](const std::string &text) {
      static constexpr char hex[] = "0123456789abcdef";
      output.push_back('"');
      for (size_t index = 0; index < text.size();) {
        const unsigned char first = static_cast<unsigned char>(text[index]);
        if (first == '"' || first == '\\') {
          output.push_back('\\');
          output.push_back(static_cast<char>(first));
          ++index;
        } else if (first < 0x20) {
          const char *escape = first == '\b'   ? "\\b"
                               : first == '\f' ? "\\f"
                               : first == '\n' ? "\\n"
                               : first == '\r' ? "\\r"
                               : first == '\t' ? "\\t"
                                               : nullptr;
          if (escape)
            output += escape;
          else {
            output += "\\u00";
            output.push_back(hex[first >> 4]);
            output.push_back(hex[first & 0x0f]);
          }
          ++index;
        } else if (first < 0x80) {
          output.push_back(static_cast<char>(first));
          ++index;
        } else {
          uint32_t codepoint = 0;
          size_t count = 0;
          if ((first & 0xe0) == 0xc0) {
            codepoint = first & 0x1f;
            count = 2;
          } else if ((first & 0xf0) == 0xe0) {
            codepoint = first & 0x0f;
            count = 3;
          } else if ((first & 0xf8) == 0xf0) {
            codepoint = first & 0x07;
            count = 4;
          } else
            throw exception("invalid UTF-8 in JSON string");
          if (index + count > text.size())
            throw exception("truncated UTF-8 in JSON string");
          for (size_t offset = 1; offset < count; ++offset) {
            const unsigned char next =
                static_cast<unsigned char>(text[index + offset]);
            if ((next & 0xc0) != 0x80)
              throw exception("invalid UTF-8 in JSON string");
            codepoint = (codepoint << 6) | (next & 0x3f);
          }
          const auto append_hex = [&output](uint16_t unit) {
            output += "\\u";
            output.push_back(hex[(unit >> 12) & 0xf]);
            output.push_back(hex[(unit >> 8) & 0xf]);
            output.push_back(hex[(unit >> 4) & 0xf]);
            output.push_back(hex[unit & 0xf]);
          };
          /* Supplementary scalars need UTF-16 pairs because JSON escapes encode 16-bit units. */
          if (codepoint <= 0xffff)
            append_hex(static_cast<uint16_t>(codepoint));
          else {
            codepoint -= 0x10000;
            append_hex(static_cast<uint16_t>(0xd800 + (codepoint >> 10)));
            append_hex(static_cast<uint16_t>(0xdc00 + (codepoint & 0x3ff)));
          }
          index += count;
        }
      }
      output.push_back('"');
    };
    const auto append = [&](auto &&self,
                            const std::shared_ptr<Node> &node) -> void {
      switch (node->kind) {
      case Node::Kind::Null:
        output += "null";
        break;
      case Node::Kind::Bool:
        output += node->boolean ? "true" : "false";
        break;
      case Node::Kind::Signed:
        output += std::to_string(node->signed_number);
        break;
      case Node::Kind::Unsigned:
        output += std::to_string(node->unsigned_number);
        break;
      case Node::Kind::Real: {
        char buffer[64];
        const auto result =
            std::to_chars(buffer, buffer + sizeof(buffer), node->real_number,
                          std::chars_format::general);
        if (result.ec != std::errc{})
          throw exception("cannot format JSON number");
        const std::string formatted(buffer, result.ptr);
        output += formatted;
        /* Preserve the Real category instead of reparsing an integer-shaped spelling as an integer. */
        if (formatted.find_first_of(".eE") == std::string::npos)
          output += ".0";
        break;
      }
      case Node::Kind::String:
        append_string(node->string);
        break;
      case Node::Kind::Array:
        output.push_back('[');
        for (size_t index = 0; index < node->array.size(); ++index) {
          if (index)
            output.push_back(',');
          self(self, node->array[index]);
        }
        output.push_back(']');
        break;
      case Node::Kind::Object: {
        output.push_back('{');
        bool first = true;
        for (const auto &item : node->object) {
          if (!first)
            output.push_back(',');
          first = false;
          append_string(item.first);
          output.push_back(':');
          self(self, item.second);
        }
        output.push_back('}');
        break;
      }
      }
    };
    append(append, node_);
    return output;
  }

  /**
   * @brief Compare tree contents rather than shared-pointer identity.
   * @param[in] left First value.
   * @param[in] right Second value.
   * @return Structural equality with numeric cross-category comparison.
   * @note Real/integer comparisons use native long double precision, not an
   * arbitrary-precision decimal equivalence relation.
   */
  friend bool operator==(const Value &left, const Value &right) {
    return equal_nodes(left.node_, right.node_);
  }
  /**
   * @brief Negate structural equality using the same numeric policy.
   * @param[in] left First value.
   * @param[in] right Second value.
   * @return True exactly when equality returns false.
   */
  friend bool operator!=(const Value &left, const Value &right) {
    return !(left == right);
  }
  /**
   * @brief Order string values only, rather than inventing an ordering across JSON kinds.
   * @param[in] left First string value.
   * @param[in] right Second string value.
   * @return Lexicographic std::string comparison result.
   * @throws exception If either operand is not a string.
   */
  friend bool operator<(const Value &left, const Value &right) {
    if (!left.is_string() || !right.is_string())
      throw exception("JSON values are not strings");
    return left.node_->string < right.node_->string;
  }

private:
  /** @brief False primary trait; unsupported conversions must not masquerade as arrays.
   * @tparam Type Candidate conversion type.
   */
  template <typename Type> struct is_std_array : std::false_type {};
  /** @brief Recognize fixed arrays so get() can require their exact length.
   * @tparam Type Array element type.
   * @tparam Count Required element count.
   */
  template <typename Type, size_t Count>
  struct is_std_array<std::array<Type, Count>> : std::true_type {};
  /** @brief False primary trait for dynamic-vector conversion detection.
   * @tparam Type Candidate conversion type.
   */
  template <typename Type> struct is_std_vector : std::false_type {};
  /** @brief Recognize vectors without fixing their element allocator.
   * @tparam Type Element type.
   * @tparam Allocator Vector allocator type.
   */
  template <typename Type, typename Allocator>
  struct is_std_vector<std::vector<Type, Allocator>> : std::true_type {};
  /**
   * @brief Construct a shared child view without invoking public deep-copy semantics.
   * @param[in] node Existing child ownership transferred into this handle.
   */
  explicit Value(std::shared_ptr<Node> node) : node_(std::move(node)) {}
  std::shared_ptr<Node> node_; /**< Current owned/shared node; may be empty after a move. */

  /**
   * @brief Recursively detach a FileNode and restore compatibility sentinel types.
   * @param[in] source OpenCV node valid for the duration of conversion.
   * @return Independent native tree, with later duplicate map keys replacing earlier ones.
   * @throws exception For unsupported nodes or invalid/out-of-range integer markers.
   */
  static Value from_node(const cv::FileNode &source);
  /**
   * @brief Adapt tokens outside strings and decode string escapes for FileStorage.
   * @param[in] input Original bytes, left unchanged.
   * @return OpenCV-readable text with typed scalar markers.
   * @throws exception For reserved raw marker text or invalid Unicode/string escapes.
   * @note This is a compatibility pass, not a complete JSON grammar validator;
   * OpenCV processes the resulting document structure.
   */
  static std::string normalize_for_opencv(const std::string &input);
  /**
   * @brief Write through FileStorage, restore marked scalars and optionally compact.
   * @param[in] value Tree to serialize.
   * @param[in] indent Negative removes insignificant whitespace; otherwise OpenCV formats.
   * @return Owned output text.
   * @throws exception For adapter-detected writer/marker failures.
   */
  static std::string write_opencv(const Value &value, int indent);
  /**
   * @brief Recursively emit containers and protect types unsupported by FileStorage.
   * @param[in,out] storage Active OpenCV JSON writer.
   * @param[in] key Name used when has_key is true.
   * @param[in] value Current value to emit.
   * @param[in] has_key Distinguishes named object members from anonymous array elements.
   * @throws exception For reserved string payloads.
   */
  static void write_node(cv::FileStorage &storage, const std::string &key,
                         const Value &value, bool has_key);
  /**
   * @brief Reject the marker namespace in a directly supplied string value.
   * @param[in] value Value checked only if its active kind is String.
   * @throws exception If that string contains the reserved prefix.
   * @note This helper is not a recursive tree or object-key validator.
   */
  static void require_unreserved(const Value &value);
  /**
   * @brief Clone child nodes recursively to implement ordinary value-copy semantics.
   * @param[in] source Non-null source node and its accessible descendants.
   * @return Independent node tree, not shared child pointers from source.
   */
  static std::shared_ptr<Node> clone(const std::shared_ptr<Node> &source);
  /**
   * @brief Compare active payloads and recursively compare container contents.
   * @param[in] left Non-null first node.
   * @param[in] right Non-null second node.
   * @return Equality under the adapter's numeric/structural comparison rules.
   */
  static bool equal_nodes(const std::shared_ptr<Node> &left,
                          const std::shared_ptr<Node> &right);
  /**
   * @brief Centralize required/optional member policy without copying a found child.
   * @param[in] key Non-null terminated member name.
   * @param[in] required Whether absence is an error instead of a fresh null value.
   * @return Shared child view, or fresh null for an optional missing member.
   * @throws exception For a nonobject or a missing required member.
   */
  Value lookup(const char *key, bool required) const;
};

inline std::shared_ptr<Node> Value::clone(const std::shared_ptr<Node> &source) {
  auto result = std::make_shared<Node>(*source);
  /* Copy scalar/string storage first, then replace copied child pointers with detached trees. */
  result->array.clear();
  result->object.clear();
  for (const auto &item : source->array)
    result->array.push_back(clone(item));
  for (const auto &item : source->object)
    result->object.emplace(item.first, clone(item.second));
  return result;
}

inline bool Value::equal_nodes(const std::shared_ptr<Node> &left,
                               const std::shared_ptr<Node> &right) {
  const auto integer = [](Node::Kind kind) {
    return kind == Node::Kind::Signed || kind == Node::Kind::Unsigned;
  };
  const auto numeric = [&](Node::Kind kind) {
    return integer(kind) || kind == Node::Kind::Real;
  };
  if (numeric(left->kind) && numeric(right->kind)) {
    /* Negative signed values must never wrap into an apparently equal unsigned magnitude. */
    if (left->kind == Node::Kind::Signed && right->kind == Node::Kind::Unsigned)
      return left->signed_number >= 0 &&
             static_cast<uint64_t>(left->signed_number) ==
                 right->unsigned_number;
    if (left->kind == Node::Kind::Unsigned && right->kind == Node::Kind::Signed)
      return right->signed_number >= 0 &&
             left->unsigned_number ==
                 static_cast<uint64_t>(right->signed_number);
    const long double left_number =
        left->kind == Node::Kind::Signed
            ? static_cast<long double>(left->signed_number)
        : left->kind == Node::Kind::Unsigned
            ? static_cast<long double>(left->unsigned_number)
            : static_cast<long double>(left->real_number);
    const long double right_number =
        right->kind == Node::Kind::Signed
            ? static_cast<long double>(right->signed_number)
        : right->kind == Node::Kind::Unsigned
            ? static_cast<long double>(right->unsigned_number)
            : static_cast<long double>(right->real_number);
    return left_number == right_number;
  }
  if (left->kind != right->kind)
    return false;
  switch (left->kind) {
  case Node::Kind::Null:
    return true;
  case Node::Kind::Bool:
    return left->boolean == right->boolean;
  case Node::Kind::Signed:
    return left->signed_number == right->signed_number;
  case Node::Kind::Unsigned:
    return left->unsigned_number == right->unsigned_number;
  case Node::Kind::Real:
    return left->real_number == right->real_number;
  case Node::Kind::String:
    return left->string == right->string;
  case Node::Kind::Array:
    if (left->array.size() != right->array.size())
      return false;
    for (size_t i = 0; i < left->array.size(); ++i)
      if (!equal_nodes(left->array[i], right->array[i]))
        return false;
    return true;
  case Node::Kind::Object:
    if (left->object.size() != right->object.size())
      return false;
    for (const auto &item : left->object) {
      auto found = right->object.find(item.first);
      if (found == right->object.end() ||
          !equal_nodes(item.second, found->second))
        return false;
    }
    return true;
  }
  return false;
}

inline Value Value::lookup(const char *key, bool required) const {
  if (!is_object())
    throw exception("JSON value is not an object");
  auto found = node_->object.find(key);
  if (found == node_->object.end()) {
    if (required)
      throw exception(std::string("missing JSON field: ") + key);
    return Value();
  }
  return Value(found->second);
}

inline void Value::require_unreserved(const Value &value) {
  if (value.is_string() &&
      value.node_->string.find("__USHER_OPENCV_JSON_") != std::string::npos)
    throw exception("JSON string uses a reserved OpenCV bridge token");
}

inline std::string Value::normalize_for_opencv(const std::string &input) {
  /* Raw marker rejection reserves an internal transport namespace, not a user data format. */
  if (input.find("__USHER_OPENCV_JSON_") != std::string::npos)
    throw exception("JSON input uses a reserved OpenCV bridge token");
  std::string output;
  output.reserve(input.size() + 32);
  bool in_string = false;
  const auto hex_value = [](char character) -> int {
    if (character >= '0' && character <= '9')
      return character - '0';
    if (character >= 'a' && character <= 'f')
      return character - 'a' + 10;
    if (character >= 'A' && character <= 'F')
      return character - 'A' + 10;
    return -1;
  };
  const auto unicode_escape = [&](size_t offset,
                                  uint32_t *codepoint) -> size_t {
    if (offset + 4 > input.size())
      throw exception("truncated JSON Unicode escape");
    uint32_t value = 0;
    for (size_t digit = 0; digit < 4; ++digit) {
      const int nibble = hex_value(input[offset + digit]);
      if (nibble < 0)
        throw exception("invalid JSON Unicode escape");
      value = (value << 4) | static_cast<uint32_t>(nibble);
    }
    *codepoint = value;
    return offset + 4;
  };
  const auto append_utf8 = [&output](uint32_t codepoint) {
    if (codepoint <= 0x7f)
      output.push_back(static_cast<char>(codepoint));
    else if (codepoint <= 0x7ff) {
      output.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else if (codepoint <= 0xffff) {
      output.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else {
      output.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    }
  };
  for (size_t i = 0; i < input.size();) {
    const char c = input[i];
    if (in_string) {
      if (c == '"') {
        in_string = false;
        output.push_back(c);
        ++i;
        continue;
      }
      if (c == '\\') {
        if (i + 1 >= input.size())
          throw exception("truncated JSON string escape");
        const char escaped = input[i + 1];
        if (escaped == '/') {
          output.push_back('/');
          i += 2;
          continue;
        }
        if (escaped == 'u') {
          uint32_t codepoint = 0;
          size_t next = unicode_escape(i + 2, &codepoint);
          /* Combine UTF-16 escapes before UTF-8 encoding; isolated halves are not scalars. */
          if (codepoint >= 0xd800 && codepoint <= 0xdbff) {
            if (next + 6 > input.size() || input[next] != '\\' ||
                input[next + 1] != 'u')
              throw exception("unpaired JSON high surrogate");
            uint32_t low = 0;
            next = unicode_escape(next + 2, &low);
            if (low < 0xdc00 || low > 0xdfff)
              throw exception("invalid JSON surrogate pair");
            codepoint = 0x10000 + ((codepoint - 0xd800) << 10) + (low - 0xdc00);
          } else if (codepoint >= 0xdc00 && codepoint <= 0xdfff) {
            throw exception("unpaired JSON low surrogate");
          }
          append_utf8(codepoint);
          i = next;
          continue;
        }
        if (std::string_view("\\\"nrtbf").find(escaped) ==
            std::string_view::npos)
          throw exception("invalid JSON string escape");
        output.push_back(c);
        output.push_back(escaped);
        i += 2;
        continue;
      }
      output.push_back(c);
      ++i;
      continue;
    }
    if (c == '"') {
      in_string = true;
      output.push_back(c);
      ++i;
      continue;
    }
    const auto token = [&](const char *text, const char *replacement) {
      const size_t length = std::char_traits<char>::length(text);
      const bool left =
          i == 0 || !(std::isalnum(static_cast<unsigned char>(input[i - 1])) ||
                      input[i - 1] == '_');
      const size_t end = i + length;
      const bool right =
          end >= input.size() ||
          !(std::isalnum(static_cast<unsigned char>(input[end])) ||
            input[end] == '_');
      if (!left || !right || input.compare(i, length, text) != 0)
        return false;
      output += "\"";
      output += replacement;
      output += "\"";
      i = end;
      return true;
    };
    /* Only outside-string tokens are rewritten; quoted ordinary words keep their meaning. */
    if (token("null", "__USHER_OPENCV_JSON_NULL__") ||
        token("true", "__USHER_OPENCV_JSON_TRUE__") ||
        token("false", "__USHER_OPENCV_JSON_FALSE__"))
      continue;
    if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
      size_t end = i;
      while (end < input.size() &&
             (std::isdigit(static_cast<unsigned char>(input[end])) ||
              input[end] == '-' || input[end] == '+' || input[end] == '.' ||
              input[end] == 'e' || input[end] == 'E'))
        ++end;
      const std::string number = input.substr(i, end - i);
      if (number.find_first_of(".eE") == std::string::npos) {
        /* Carry integer digits as text so FileStorage cannot narrow them before from_chars. */
        const char *prefix = !number.empty() && number[0] == '-'
                                 ? "\"__USHER_OPENCV_JSON_INT__"
                                 : "\"__USHER_OPENCV_JSON_UINT__";
        output += prefix;
        output += number;
        output += "__\"";
        i = end;
        continue;
      }
      output += number;
      i = end;
      continue;
    }
    output.push_back(c);
    ++i;
  }
  return output;
}

inline Value Value::from_node(const cv::FileNode &source) {
  if (source.isMap()) {
    Value result = object();
    for (auto iterator = source.begin(); iterator != source.end(); ++iterator) {
      const cv::FileNode child = *iterator;
      result.set(child.name(), from_node(child));
    }
    return result;
  }
  if (source.isSeq()) {
    Value result = array();
    for (auto iterator = source.begin(); iterator != source.end(); ++iterator)
      result.push_back(from_node(*iterator));
    return result;
  }
  if (source.isString()) {
    const std::string text = static_cast<std::string>(source);
    if (text == "__USHER_OPENCV_JSON_NULL__")
      return Value();
    if (text == "__USHER_OPENCV_JSON_TRUE__")
      return Value(true);
    if (text == "__USHER_OPENCV_JSON_FALSE__")
      return Value(false);
    const auto parse_integer = [&text](const char *prefix,
                                       bool is_unsigned) -> Value {
      const size_t prefix_length = std::char_traits<char>::length(prefix);
      if (text.compare(0, prefix_length, prefix) != 0 ||
          text.size() < prefix_length + 2 ||
          text.compare(text.size() - 2, 2, "__") != 0)
        return Value(text);
      const std::string digits =
          text.substr(prefix_length, text.size() - prefix_length - 2);
        /* Require full consumption as well as success; accepting a numeric prefix hides junk. */
      if (is_unsigned) {
        uint64_t number = 0;
        const auto result = std::from_chars(
            digits.data(), digits.data() + digits.size(), number);
        if (result.ec != std::errc{} ||
            result.ptr != digits.data() + digits.size())
          throw exception("invalid unsigned JSON integer");
        return Value(number);
      }
      int64_t number = 0;
      const auto result =
          std::from_chars(digits.data(), digits.data() + digits.size(), number);
      if (result.ec != std::errc{} ||
          result.ptr != digits.data() + digits.size())
        throw exception("invalid signed JSON integer");
      return Value(number);
    };
    if (text.compare(0, 26, "__USHER_OPENCV_JSON_UINT__") == 0)
      return parse_integer("__USHER_OPENCV_JSON_UINT__", true);
    if (text.compare(0, 25, "__USHER_OPENCV_JSON_INT__") == 0)
      return parse_integer("__USHER_OPENCV_JSON_INT__", false);
    return Value(text);
  }
  if (source.isInt())
    return Value(static_cast<int>(source));
  if (source.isReal())
    return Value(static_cast<double>(source));
  throw exception("unsupported OpenCV JSON value type");
}

inline void Value::write_node(cv::FileStorage &storage, const std::string &key,
                              const Value &value, bool has_key) {
  const auto write_key = [&] {
    if (has_key)
      storage << key;
  };
  switch (value.node_->kind) {
  case Node::Kind::Null:
    write_key();
    storage << "__USHER_OPENCV_JSON_NULL__";
    return;
  case Node::Kind::Bool:
    write_key();
    storage << (value.node_->boolean ? "__USHER_OPENCV_JSON_TRUE__"
                                     : "__USHER_OPENCV_JSON_FALSE__");
    return;
  case Node::Kind::Signed:
    write_key();
    /* Mark both integer widths instead of depending on FileStorage's integer overload range. */
    storage << ("__USHER_OPENCV_JSON_INT__" +
                std::to_string(value.node_->signed_number) + "__");
    return;
  case Node::Kind::Unsigned:
    write_key();
    storage << ("__USHER_OPENCV_JSON_UINT__" +
                std::to_string(value.node_->unsigned_number) + "__");
    return;
  case Node::Kind::Real:
    write_key();
    storage << value.node_->real_number;
    return;
  case Node::Kind::String:
    require_unreserved(value);
    write_key();
    storage << value.node_->string;
    return;
  case Node::Kind::Object:
    storage.startWriteStruct(has_key ? key : "", cv::FileNode::MAP);
    for (const auto &item : value.node_->object)
      write_node(storage, item.first, Value(item.second), true);
    storage.endWriteStruct();
    return;
  case Node::Kind::Array:
    storage.startWriteStruct(has_key ? key : "", cv::FileNode::SEQ);
    for (const auto &item : value.node_->array)
      write_node(storage, "", Value(item), false);
    storage.endWriteStruct();
    return;
  }
}

inline std::string Value::write_opencv(const Value &value, int indent) {
  cv::FileStorage storage("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                                  cv::FileStorage::FORMAT_JSON);
  if (!storage.isOpened())
    throw exception("cannot open OpenCV JSON writer");
  if (value.is_object()) {
    for (const auto &item : value.node_->object)
      write_node(storage, item.first, Value(item.second), true);
  } else {
    write_node(storage, "", value, false);
  }
  std::string output = storage.releaseAndGetString();
  /* Restore markers after serialization, when OpenCV has finished quoting transport strings. */
  const auto replace_token = [&output](const std::string &marker,
                                       const std::string &replacement) {
    const std::string quoted = "\"" + marker + "\"";
    size_t position = 0;
    while ((position = output.find(quoted, position)) != std::string::npos) {
      output.replace(position, quoted.size(), replacement);
      position += replacement.size();
    }
  };
  replace_token("__USHER_OPENCV_JSON_NULL__", "null");
  replace_token("__USHER_OPENCV_JSON_TRUE__", "true");
  replace_token("__USHER_OPENCV_JSON_FALSE__", "false");
  const auto replace_integer = [&output](const char *prefix) {
    size_t position = 0;
    while ((position = output.find(prefix, position)) != std::string::npos) {
      const size_t start = position;
      const size_t digits = position + std::char_traits<char>::length(prefix);
      const size_t end = output.find("__\"", digits);
      if (end == std::string::npos)
        break;
      const size_t quoted_start = start - 1;
      output.replace(quoted_start, end + 3 - quoted_start,
                     output.substr(digits, end - digits));
      position = quoted_start + 1;
    }
  };
  replace_integer("__USHER_OPENCV_JSON_INT__");
  replace_integer("__USHER_OPENCV_JSON_UINT__");
  if (indent >= 0)
    return output;
  std::string compact;
  compact.reserve(output.size());
  /* Whitespace inside quoted strings is data; track escaped quotes before removing trivia. */
  bool in_string = false, escaped = false;
  for (char character : output) {
    if (in_string) {
      compact.push_back(character);
      if (escaped)
        escaped = false;
      else if (character == '\\')
        escaped = true;
      else if (character == '"')
        in_string = false;
    } else if (character == '"') {
      in_string = true;
      compact.push_back(character);
    } else if (character != ' ' && character != '\t' && character != '\n' &&
               character != '\r') {
      compact.push_back(character);
    }
  }
  return compact;
}

} // namespace usher_cv_json

#endif
