// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/common/proto/ceres_solver_options.proto

#include "cartographer/common/proto/ceres_solver_options.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

namespace cartographer {
namespace common {
namespace proto {
class CeresSolverOptionsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<CeresSolverOptions> _instance;
} _CeresSolverOptions_default_instance_;
}  // namespace proto
}  // namespace common
}  // namespace cartographer
static void InitDefaultsCeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::common::proto::_CeresSolverOptions_default_instance_;
    new (ptr) ::cartographer::common::proto::CeresSolverOptions();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::common::proto::CeresSolverOptions::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_CeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsCeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto}, {}};

void InitDefaults_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_CeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto[1];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::common::proto::CeresSolverOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::common::proto::CeresSolverOptions, use_nonmonotonic_steps_),
  PROTOBUF_FIELD_OFFSET(::cartographer::common::proto::CeresSolverOptions, max_num_iterations_),
  PROTOBUF_FIELD_OFFSET(::cartographer::common::proto::CeresSolverOptions, num_threads_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::common::proto::CeresSolverOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::common::proto::_CeresSolverOptions_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto = {
  {}, AddDescriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, "cartographer/common/proto/ceres_solver_options.proto", schemas,
  file_default_instances, TableStruct_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto::offsets,
  file_level_metadata_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, 1, file_level_enum_descriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, file_level_service_descriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto,
};

const char descriptor_table_protodef_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto[] =
  "\n4cartographer/common/proto/ceres_solver"
  "_options.proto\022\031cartographer.common.prot"
  "o\"e\n\022CeresSolverOptions\022\036\n\026use_nonmonoto"
  "nic_steps\030\001 \001(\010\022\032\n\022max_num_iterations\030\002 "
  "\001(\005\022\023\n\013num_threads\030\003 \001(\005b\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto = {
  false, InitDefaults_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, 
  descriptor_table_protodef_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto,
  "cartographer/common/proto/ceres_solver_options.proto", &assign_descriptors_table_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, 192,
};

void AddDescriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto, deps, 0);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto = []() { AddDescriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto(); return true; }();
namespace cartographer {
namespace common {
namespace proto {

// ===================================================================

void CeresSolverOptions::InitAsDefaultInstance() {
}
class CeresSolverOptions::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CeresSolverOptions::kUseNonmonotonicStepsFieldNumber;
const int CeresSolverOptions::kMaxNumIterationsFieldNumber;
const int CeresSolverOptions::kNumThreadsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CeresSolverOptions::CeresSolverOptions()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.common.proto.CeresSolverOptions)
}
CeresSolverOptions::CeresSolverOptions(const CeresSolverOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&use_nonmonotonic_steps_, &from.use_nonmonotonic_steps_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_threads_) -
    reinterpret_cast<char*>(&use_nonmonotonic_steps_)) + sizeof(num_threads_));
  // @@protoc_insertion_point(copy_constructor:cartographer.common.proto.CeresSolverOptions)
}

void CeresSolverOptions::SharedCtor() {
  ::memset(&use_nonmonotonic_steps_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_threads_) -
      reinterpret_cast<char*>(&use_nonmonotonic_steps_)) + sizeof(num_threads_));
}

CeresSolverOptions::~CeresSolverOptions() {
  // @@protoc_insertion_point(destructor:cartographer.common.proto.CeresSolverOptions)
  SharedDtor();
}

void CeresSolverOptions::SharedDtor() {
}

void CeresSolverOptions::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CeresSolverOptions& CeresSolverOptions::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_CeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto.base);
  return *internal_default_instance();
}


void CeresSolverOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.common.proto.CeresSolverOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&use_nonmonotonic_steps_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_threads_) -
      reinterpret_cast<char*>(&use_nonmonotonic_steps_)) + sizeof(num_threads_));
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* CeresSolverOptions::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<CeresSolverOptions*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // bool use_nonmonotonic_steps = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 8) goto handle_unusual;
        msg->set_use_nonmonotonic_steps(::google::protobuf::internal::ReadVarint(&ptr));
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        break;
      }
      // int32 max_num_iterations = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 16) goto handle_unusual;
        msg->set_max_num_iterations(::google::protobuf::internal::ReadVarint(&ptr));
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        break;
      }
      // int32 num_threads = 3;
      case 3: {
        if (static_cast<::google::protobuf::uint8>(tag) != 24) goto handle_unusual;
        msg->set_num_threads(::google::protobuf::internal::ReadVarint(&ptr));
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool CeresSolverOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.common.proto.CeresSolverOptions)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // bool use_nonmonotonic_steps = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (8 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &use_nonmonotonic_steps_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 max_num_iterations = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (16 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &max_num_iterations_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 num_threads = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (24 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_threads_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.common.proto.CeresSolverOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.common.proto.CeresSolverOptions)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void CeresSolverOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.common.proto.CeresSolverOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bool use_nonmonotonic_steps = 1;
  if (this->use_nonmonotonic_steps() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(1, this->use_nonmonotonic_steps(), output);
  }

  // int32 max_num_iterations = 2;
  if (this->max_num_iterations() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->max_num_iterations(), output);
  }

  // int32 num_threads = 3;
  if (this->num_threads() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->num_threads(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.common.proto.CeresSolverOptions)
}

::google::protobuf::uint8* CeresSolverOptions::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.common.proto.CeresSolverOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bool use_nonmonotonic_steps = 1;
  if (this->use_nonmonotonic_steps() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(1, this->use_nonmonotonic_steps(), target);
  }

  // int32 max_num_iterations = 2;
  if (this->max_num_iterations() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->max_num_iterations(), target);
  }

  // int32 num_threads = 3;
  if (this->num_threads() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->num_threads(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.common.proto.CeresSolverOptions)
  return target;
}

size_t CeresSolverOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.common.proto.CeresSolverOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // bool use_nonmonotonic_steps = 1;
  if (this->use_nonmonotonic_steps() != 0) {
    total_size += 1 + 1;
  }

  // int32 max_num_iterations = 2;
  if (this->max_num_iterations() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->max_num_iterations());
  }

  // int32 num_threads = 3;
  if (this->num_threads() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->num_threads());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CeresSolverOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.common.proto.CeresSolverOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const CeresSolverOptions* source =
      ::google::protobuf::DynamicCastToGenerated<CeresSolverOptions>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.common.proto.CeresSolverOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.common.proto.CeresSolverOptions)
    MergeFrom(*source);
  }
}

void CeresSolverOptions::MergeFrom(const CeresSolverOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.common.proto.CeresSolverOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.use_nonmonotonic_steps() != 0) {
    set_use_nonmonotonic_steps(from.use_nonmonotonic_steps());
  }
  if (from.max_num_iterations() != 0) {
    set_max_num_iterations(from.max_num_iterations());
  }
  if (from.num_threads() != 0) {
    set_num_threads(from.num_threads());
  }
}

void CeresSolverOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.common.proto.CeresSolverOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CeresSolverOptions::CopyFrom(const CeresSolverOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.common.proto.CeresSolverOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CeresSolverOptions::IsInitialized() const {
  return true;
}

void CeresSolverOptions::Swap(CeresSolverOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CeresSolverOptions::InternalSwap(CeresSolverOptions* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(use_nonmonotonic_steps_, other->use_nonmonotonic_steps_);
  swap(max_num_iterations_, other->max_num_iterations_);
  swap(num_threads_, other->num_threads_);
}

::google::protobuf::Metadata CeresSolverOptions::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto);
  return ::file_level_metadata_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace common
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::cartographer::common::proto::CeresSolverOptions* Arena::CreateMaybeMessage< ::cartographer::common::proto::CeresSolverOptions >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::common::proto::CeresSolverOptions >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>