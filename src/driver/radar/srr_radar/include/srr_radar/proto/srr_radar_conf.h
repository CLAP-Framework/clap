// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: srr_radar_conf.proto

#ifndef PROTOBUF_srr_5fradar_5fconf_2eproto__INCLUDED
#define PROTOBUF_srr_5fradar_5fconf_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace drivers {
namespace srr_radar {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_srr_5fradar_5fconf_2eproto();
void protobuf_AssignDesc_srr_5fradar_5fconf_2eproto();
void protobuf_ShutdownFile_srr_5fradar_5fconf_2eproto();

class RadarConf;
class SrrRadarConf;

enum OutputType {
  OUTPUT_TYPE_TRACKS = 0,
  OUTPUT_TYPE_CLUSTERS = 1,
  OUTPUT_TYPE_ERROR = 2
};
bool OutputType_IsValid(int value);
const OutputType OutputType_MIN = OUTPUT_TYPE_TRACKS;
const OutputType OutputType_MAX = OUTPUT_TYPE_ERROR;
const int OutputType_ARRAYSIZE = OutputType_MAX + 1;

const ::google::protobuf::EnumDescriptor* OutputType_descriptor();
inline const ::std::string& OutputType_Name(OutputType value) {
  return ::google::protobuf::internal::NameOfEnum(
    OutputType_descriptor(), value);
}
inline bool OutputType_Parse(
    const ::std::string& name, OutputType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<OutputType>(
    OutputType_descriptor(), name, value);
}
// ===================================================================

class RadarConf : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:drivers.srr_radar.RadarConf) */ {
 public:
  RadarConf();
  virtual ~RadarConf();

  RadarConf(const RadarConf& from);

  inline RadarConf& operator=(const RadarConf& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const RadarConf& default_instance();

  void Swap(RadarConf* other);

  // implements Message ----------------------------------------------

  inline RadarConf* New() const { return New(NULL); }

  RadarConf* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RadarConf& from);
  void MergeFrom(const RadarConf& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(RadarConf* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional bool sensor_id_valid = 1 [default = false];
  bool has_sensor_id_valid() const;
  void clear_sensor_id_valid();
  static const int kSensorIdValidFieldNumber = 1;
  bool sensor_id_valid() const;
  void set_sensor_id_valid(bool value);

  // optional bool output_type_valid = 2 [default = true];
  bool has_output_type_valid() const;
  void clear_output_type_valid();
  static const int kOutputTypeValidFieldNumber = 2;
  bool output_type_valid() const;
  void set_output_type_valid(bool value);

  // optional uint32 sensor_id = 3 [default = 0];
  bool has_sensor_id() const;
  void clear_sensor_id();
  static const int kSensorIdFieldNumber = 3;
  ::google::protobuf::uint32 sensor_id() const;
  void set_sensor_id(::google::protobuf::uint32 value);

  // optional .drivers.srr_radar.OutputType output_type = 4 [default = OUTPUT_TYPE_CLUSTERS];
  bool has_output_type() const;
  void clear_output_type();
  static const int kOutputTypeFieldNumber = 4;
  ::drivers::srr_radar::OutputType output_type() const;
  void set_output_type(::drivers::srr_radar::OutputType value);

  // @@protoc_insertion_point(class_scope:drivers.srr_radar.RadarConf)
 private:
  inline void set_has_sensor_id_valid();
  inline void clear_has_sensor_id_valid();
  inline void set_has_output_type_valid();
  inline void clear_has_output_type_valid();
  inline void set_has_sensor_id();
  inline void clear_has_sensor_id();
  inline void set_has_output_type();
  inline void clear_has_output_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  bool sensor_id_valid_;
  bool output_type_valid_;
  ::google::protobuf::uint32 sensor_id_;
  int output_type_;
  friend void  protobuf_AddDesc_srr_5fradar_5fconf_2eproto();
  friend void protobuf_AssignDesc_srr_5fradar_5fconf_2eproto();
  friend void protobuf_ShutdownFile_srr_5fradar_5fconf_2eproto();

  void InitAsDefaultInstance();
  static RadarConf* default_instance_;
};
// -------------------------------------------------------------------

class SrrRadarConf : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:drivers.srr_radar.SrrRadarConf) */ {
 public:
  SrrRadarConf();
  virtual ~SrrRadarConf();

  SrrRadarConf(const SrrRadarConf& from);

  inline SrrRadarConf& operator=(const SrrRadarConf& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SrrRadarConf& default_instance();

  void Swap(SrrRadarConf* other);

  // implements Message ----------------------------------------------

  inline SrrRadarConf* New() const { return New(NULL); }

  SrrRadarConf* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SrrRadarConf& from);
  void MergeFrom(const SrrRadarConf& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(SrrRadarConf* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .drivers.srr_radar.RadarConf radar_conf = 1;
  bool has_radar_conf() const;
  void clear_radar_conf();
  static const int kRadarConfFieldNumber = 1;
  const ::drivers::srr_radar::RadarConf& radar_conf() const;
  ::drivers::srr_radar::RadarConf* mutable_radar_conf();
  ::drivers::srr_radar::RadarConf* release_radar_conf();
  void set_allocated_radar_conf(::drivers::srr_radar::RadarConf* radar_conf);

  // optional string radar_channel = 2;
  bool has_radar_channel() const;
  void clear_radar_channel();
  static const int kRadarChannelFieldNumber = 2;
  const ::std::string& radar_channel() const;
  void set_radar_channel(const ::std::string& value);
  void set_radar_channel(const char* value);
  void set_radar_channel(const char* value, size_t size);
  ::std::string* mutable_radar_channel();
  ::std::string* release_radar_channel();
  void set_allocated_radar_channel(::std::string* radar_channel);

  // @@protoc_insertion_point(class_scope:drivers.srr_radar.SrrRadarConf)
 private:
  inline void set_has_radar_conf();
  inline void clear_has_radar_conf();
  inline void set_has_radar_channel();
  inline void clear_has_radar_channel();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::drivers::srr_radar::RadarConf* radar_conf_;
  ::google::protobuf::internal::ArenaStringPtr radar_channel_;
  friend void  protobuf_AddDesc_srr_5fradar_5fconf_2eproto();
  friend void protobuf_AssignDesc_srr_5fradar_5fconf_2eproto();
  friend void protobuf_ShutdownFile_srr_5fradar_5fconf_2eproto();

  void InitAsDefaultInstance();
  static SrrRadarConf* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// RadarConf

// optional bool sensor_id_valid = 1 [default = false];
inline bool RadarConf::has_sensor_id_valid() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RadarConf::set_has_sensor_id_valid() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RadarConf::clear_has_sensor_id_valid() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RadarConf::clear_sensor_id_valid() {
  sensor_id_valid_ = false;
  clear_has_sensor_id_valid();
}
inline bool RadarConf::sensor_id_valid() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.RadarConf.sensor_id_valid)
  return sensor_id_valid_;
}
inline void RadarConf::set_sensor_id_valid(bool value) {
  set_has_sensor_id_valid();
  sensor_id_valid_ = value;
  // @@protoc_insertion_point(field_set:drivers.srr_radar.RadarConf.sensor_id_valid)
}

// optional bool output_type_valid = 2 [default = true];
inline bool RadarConf::has_output_type_valid() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RadarConf::set_has_output_type_valid() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RadarConf::clear_has_output_type_valid() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RadarConf::clear_output_type_valid() {
  output_type_valid_ = true;
  clear_has_output_type_valid();
}
inline bool RadarConf::output_type_valid() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.RadarConf.output_type_valid)
  return output_type_valid_;
}
inline void RadarConf::set_output_type_valid(bool value) {
  set_has_output_type_valid();
  output_type_valid_ = value;
  // @@protoc_insertion_point(field_set:drivers.srr_radar.RadarConf.output_type_valid)
}

// optional uint32 sensor_id = 3 [default = 0];
inline bool RadarConf::has_sensor_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void RadarConf::set_has_sensor_id() {
  _has_bits_[0] |= 0x00000004u;
}
inline void RadarConf::clear_has_sensor_id() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void RadarConf::clear_sensor_id() {
  sensor_id_ = 0u;
  clear_has_sensor_id();
}
inline ::google::protobuf::uint32 RadarConf::sensor_id() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.RadarConf.sensor_id)
  return sensor_id_;
}
inline void RadarConf::set_sensor_id(::google::protobuf::uint32 value) {
  set_has_sensor_id();
  sensor_id_ = value;
  // @@protoc_insertion_point(field_set:drivers.srr_radar.RadarConf.sensor_id)
}

// optional .drivers.srr_radar.OutputType output_type = 4 [default = OUTPUT_TYPE_CLUSTERS];
inline bool RadarConf::has_output_type() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void RadarConf::set_has_output_type() {
  _has_bits_[0] |= 0x00000008u;
}
inline void RadarConf::clear_has_output_type() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void RadarConf::clear_output_type() {
  output_type_ = 1;
  clear_has_output_type();
}
inline ::drivers::srr_radar::OutputType RadarConf::output_type() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.RadarConf.output_type)
  return static_cast< ::drivers::srr_radar::OutputType >(output_type_);
}
inline void RadarConf::set_output_type(::drivers::srr_radar::OutputType value) {
  assert(::drivers::srr_radar::OutputType_IsValid(value));
  set_has_output_type();
  output_type_ = value;
  // @@protoc_insertion_point(field_set:drivers.srr_radar.RadarConf.output_type)
}

// -------------------------------------------------------------------

// SrrRadarConf

// optional .drivers.srr_radar.RadarConf radar_conf = 1;
inline bool SrrRadarConf::has_radar_conf() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SrrRadarConf::set_has_radar_conf() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SrrRadarConf::clear_has_radar_conf() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SrrRadarConf::clear_radar_conf() {
  if (radar_conf_ != NULL) radar_conf_->::drivers::srr_radar::RadarConf::Clear();
  clear_has_radar_conf();
}
inline const ::drivers::srr_radar::RadarConf& SrrRadarConf::radar_conf() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.SrrRadarConf.radar_conf)
  return radar_conf_ != NULL ? *radar_conf_ : *default_instance_->radar_conf_;
}
inline ::drivers::srr_radar::RadarConf* SrrRadarConf::mutable_radar_conf() {
  set_has_radar_conf();
  if (radar_conf_ == NULL) {
    radar_conf_ = new ::drivers::srr_radar::RadarConf;
  }
  // @@protoc_insertion_point(field_mutable:drivers.srr_radar.SrrRadarConf.radar_conf)
  return radar_conf_;
}
inline ::drivers::srr_radar::RadarConf* SrrRadarConf::release_radar_conf() {
  // @@protoc_insertion_point(field_release:drivers.srr_radar.SrrRadarConf.radar_conf)
  clear_has_radar_conf();
  ::drivers::srr_radar::RadarConf* temp = radar_conf_;
  radar_conf_ = NULL;
  return temp;
}
inline void SrrRadarConf::set_allocated_radar_conf(::drivers::srr_radar::RadarConf* radar_conf) {
  delete radar_conf_;
  radar_conf_ = radar_conf;
  if (radar_conf) {
    set_has_radar_conf();
  } else {
    clear_has_radar_conf();
  }
  // @@protoc_insertion_point(field_set_allocated:drivers.srr_radar.SrrRadarConf.radar_conf)
}

// optional string radar_channel = 2;
inline bool SrrRadarConf::has_radar_channel() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SrrRadarConf::set_has_radar_channel() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SrrRadarConf::clear_has_radar_channel() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SrrRadarConf::clear_radar_channel() {
  radar_channel_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_radar_channel();
}
inline const ::std::string& SrrRadarConf::radar_channel() const {
  // @@protoc_insertion_point(field_get:drivers.srr_radar.SrrRadarConf.radar_channel)
  return radar_channel_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SrrRadarConf::set_radar_channel(const ::std::string& value) {
  set_has_radar_channel();
  radar_channel_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:drivers.srr_radar.SrrRadarConf.radar_channel)
}
inline void SrrRadarConf::set_radar_channel(const char* value) {
  set_has_radar_channel();
  radar_channel_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:drivers.srr_radar.SrrRadarConf.radar_channel)
}
inline void SrrRadarConf::set_radar_channel(const char* value, size_t size) {
  set_has_radar_channel();
  radar_channel_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:drivers.srr_radar.SrrRadarConf.radar_channel)
}
inline ::std::string* SrrRadarConf::mutable_radar_channel() {
  set_has_radar_channel();
  // @@protoc_insertion_point(field_mutable:drivers.srr_radar.SrrRadarConf.radar_channel)
  return radar_channel_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SrrRadarConf::release_radar_channel() {
  // @@protoc_insertion_point(field_release:drivers.srr_radar.SrrRadarConf.radar_channel)
  clear_has_radar_channel();
  return radar_channel_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SrrRadarConf::set_allocated_radar_channel(::std::string* radar_channel) {
  if (radar_channel != NULL) {
    set_has_radar_channel();
  } else {
    clear_has_radar_channel();
  }
  radar_channel_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), radar_channel);
  // @@protoc_insertion_point(field_set_allocated:drivers.srr_radar.SrrRadarConf.radar_channel)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace srr_radar
}  // namespace drivers

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::drivers::srr_radar::OutputType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::drivers::srr_radar::OutputType>() {
  return ::drivers::srr_radar::OutputType_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_srr_5fradar_5fconf_2eproto__INCLUDED