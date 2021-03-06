// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.proto

#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"

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

extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_CeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto;
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {
class CeresScanMatcherOptions3DDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<CeresScanMatcherOptions3D> _instance;
} _CeresScanMatcherOptions3D_default_instance_;
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsCeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions3D_default_instance_;
    new (ptr) ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_CeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsCeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto}, {
      &scc_info_CeresSolverOptions_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto.base,}};

void InitDefaults_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_CeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto[1];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, occupied_space_weight_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, translation_weight_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, rotation_weight_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, only_optimize_yaw_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D, ceres_solver_options_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions3D_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto = {
  {}, AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.proto", schemas,
  file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto,
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto[] =
  "\nLcartographer/mapping/proto/scan_matchi"
  "ng/ceres_scan_matcher_options_3d.proto\022("
  "cartographer.mapping.scan_matching.proto"
  "\0324cartographer/common/proto/ceres_solver"
  "_options.proto\"\327\001\n\031CeresScanMatcherOptio"
  "ns3D\022\035\n\025occupied_space_weight\030\001 \003(\001\022\032\n\022t"
  "ranslation_weight\030\002 \001(\001\022\027\n\017rotation_weig"
  "ht\030\003 \001(\001\022\031\n\021only_optimize_yaw\030\005 \001(\010\022K\n\024c"
  "eres_solver_options\030\006 \001(\0132-.cartographer"
  ".common.proto.CeresSolverOptionsb\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto = {
  false, InitDefaults_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, 
  descriptor_table_protodef_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto,
  "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.proto", &assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, 400,
};

void AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
    ::AddDescriptors_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto,
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto, deps, 1);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto = []() { AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto(); return true; }();
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {

// ===================================================================

void CeresScanMatcherOptions3D::InitAsDefaultInstance() {
  ::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions3D_default_instance_._instance.get_mutable()->ceres_solver_options_ = const_cast< ::cartographer::common::proto::CeresSolverOptions*>(
      ::cartographer::common::proto::CeresSolverOptions::internal_default_instance());
}
class CeresScanMatcherOptions3D::HasBitSetters {
 public:
  static const ::cartographer::common::proto::CeresSolverOptions& ceres_solver_options(const CeresScanMatcherOptions3D* msg);
};

const ::cartographer::common::proto::CeresSolverOptions&
CeresScanMatcherOptions3D::HasBitSetters::ceres_solver_options(const CeresScanMatcherOptions3D* msg) {
  return *msg->ceres_solver_options_;
}
void CeresScanMatcherOptions3D::clear_ceres_solver_options() {
  if (GetArenaNoVirtual() == nullptr && ceres_solver_options_ != nullptr) {
    delete ceres_solver_options_;
  }
  ceres_solver_options_ = nullptr;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CeresScanMatcherOptions3D::kOccupiedSpaceWeightFieldNumber;
const int CeresScanMatcherOptions3D::kTranslationWeightFieldNumber;
const int CeresScanMatcherOptions3D::kRotationWeightFieldNumber;
const int CeresScanMatcherOptions3D::kOnlyOptimizeYawFieldNumber;
const int CeresScanMatcherOptions3D::kCeresSolverOptionsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CeresScanMatcherOptions3D::CeresScanMatcherOptions3D()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
}
CeresScanMatcherOptions3D::CeresScanMatcherOptions3D(const CeresScanMatcherOptions3D& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr),
      occupied_space_weight_(from.occupied_space_weight_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_ceres_solver_options()) {
    ceres_solver_options_ = new ::cartographer::common::proto::CeresSolverOptions(*from.ceres_solver_options_);
  } else {
    ceres_solver_options_ = nullptr;
  }
  ::memcpy(&translation_weight_, &from.translation_weight_,
    static_cast<size_t>(reinterpret_cast<char*>(&only_optimize_yaw_) -
    reinterpret_cast<char*>(&translation_weight_)) + sizeof(only_optimize_yaw_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
}

void CeresScanMatcherOptions3D::SharedCtor() {
  ::google::protobuf::internal::InitSCC(
      &scc_info_CeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto.base);
  ::memset(&ceres_solver_options_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&only_optimize_yaw_) -
      reinterpret_cast<char*>(&ceres_solver_options_)) + sizeof(only_optimize_yaw_));
}

CeresScanMatcherOptions3D::~CeresScanMatcherOptions3D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  SharedDtor();
}

void CeresScanMatcherOptions3D::SharedDtor() {
  if (this != internal_default_instance()) delete ceres_solver_options_;
}

void CeresScanMatcherOptions3D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CeresScanMatcherOptions3D& CeresScanMatcherOptions3D::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_CeresScanMatcherOptions3D_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto.base);
  return *internal_default_instance();
}


void CeresScanMatcherOptions3D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  occupied_space_weight_.Clear();
  if (GetArenaNoVirtual() == nullptr && ceres_solver_options_ != nullptr) {
    delete ceres_solver_options_;
  }
  ceres_solver_options_ = nullptr;
  ::memset(&translation_weight_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&only_optimize_yaw_) -
      reinterpret_cast<char*>(&translation_weight_)) + sizeof(only_optimize_yaw_));
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* CeresScanMatcherOptions3D::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<CeresScanMatcherOptions3D*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // repeated double occupied_space_weight = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) == 10) {
          ptr = ::google::protobuf::io::ReadSize(ptr, &size);
          GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
          parser_till_end = ::google::protobuf::internal::PackedDoubleParser;
          object = msg->mutable_occupied_space_weight();
          if (size > end - ptr) goto len_delim_till_end;
          auto newend = ptr + size;
          if (size) ptr = parser_till_end(ptr, newend, object, ctx);
          GOOGLE_PROTOBUF_PARSER_ASSERT(ptr == newend);
          break;
        } else if (static_cast<::google::protobuf::uint8>(tag) != 9) goto handle_unusual;
        do {
          msg->add_occupied_space_weight(::google::protobuf::io::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
          if (ptr >= end) break;
        } while ((::google::protobuf::io::UnalignedLoad<::google::protobuf::uint64>(ptr) & 255) == 9 && (ptr += 1));
        break;
      }
      // double translation_weight = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 17) goto handle_unusual;
        msg->set_translation_weight(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double rotation_weight = 3;
      case 3: {
        if (static_cast<::google::protobuf::uint8>(tag) != 25) goto handle_unusual;
        msg->set_rotation_weight(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // bool only_optimize_yaw = 5;
      case 5: {
        if (static_cast<::google::protobuf::uint8>(tag) != 40) goto handle_unusual;
        msg->set_only_optimize_yaw(::google::protobuf::internal::ReadVarint(&ptr));
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        break;
      }
      // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 6;
      case 6: {
        if (static_cast<::google::protobuf::uint8>(tag) != 50) goto handle_unusual;
        ptr = ::google::protobuf::io::ReadSize(ptr, &size);
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        parser_till_end = ::cartographer::common::proto::CeresSolverOptions::_InternalParse;
        object = msg->mutable_ceres_solver_options();
        if (size > end - ptr) goto len_delim_till_end;
        ptr += size;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ctx->ParseExactRange(
            {parser_till_end, object}, ptr - size, ptr));
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
len_delim_till_end:
  return ctx->StoreAndTailCall(ptr, end, {_InternalParse, msg},
                               {parser_till_end, object}, size);
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool CeresScanMatcherOptions3D::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated double occupied_space_weight = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (10 & 0xFF)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_occupied_space_weight())));
        } else if (static_cast< ::google::protobuf::uint8>(tag) == (9 & 0xFF)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 10u, input, this->mutable_occupied_space_weight())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double translation_weight = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (17 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &translation_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double rotation_weight = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (25 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &rotation_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bool only_optimize_yaw = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (40 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &only_optimize_yaw_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (50 & 0xFF)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_ceres_solver_options()));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void CeresScanMatcherOptions3D::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double occupied_space_weight = 1;
  if (this->occupied_space_weight_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(1, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_occupied_space_weight_cached_byte_size_.load(
        std::memory_order_relaxed));
    ::google::protobuf::internal::WireFormatLite::WriteDoubleArray(
      this->occupied_space_weight().data(), this->occupied_space_weight_size(), output);
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->translation_weight(), output);
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->rotation_weight(), output);
  }

  // bool only_optimize_yaw = 5;
  if (this->only_optimize_yaw() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(5, this->only_optimize_yaw(), output);
  }

  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 6;
  if (this->has_ceres_solver_options()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      6, HasBitSetters::ceres_solver_options(this), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
}

::google::protobuf::uint8* CeresScanMatcherOptions3D::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double occupied_space_weight = 1;
  if (this->occupied_space_weight_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      1,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        _occupied_space_weight_cached_byte_size_.load(std::memory_order_relaxed),
         target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleNoTagToArray(this->occupied_space_weight_, target);
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->translation_weight(), target);
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->rotation_weight(), target);
  }

  // bool only_optimize_yaw = 5;
  if (this->only_optimize_yaw() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(5, this->only_optimize_yaw(), target);
  }

  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 6;
  if (this->has_ceres_solver_options()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        6, HasBitSetters::ceres_solver_options(this), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  return target;
}

size_t CeresScanMatcherOptions3D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double occupied_space_weight = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->occupied_space_weight_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast<::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    _occupied_space_weight_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 6;
  if (this->has_ceres_solver_options()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *ceres_solver_options_);
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    total_size += 1 + 8;
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    total_size += 1 + 8;
  }

  // bool only_optimize_yaw = 5;
  if (this->only_optimize_yaw() != 0) {
    total_size += 1 + 1;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CeresScanMatcherOptions3D::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  GOOGLE_DCHECK_NE(&from, this);
  const CeresScanMatcherOptions3D* source =
      ::google::protobuf::DynamicCastToGenerated<CeresScanMatcherOptions3D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
    MergeFrom(*source);
  }
}

void CeresScanMatcherOptions3D::MergeFrom(const CeresScanMatcherOptions3D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  occupied_space_weight_.MergeFrom(from.occupied_space_weight_);
  if (from.has_ceres_solver_options()) {
    mutable_ceres_solver_options()->::cartographer::common::proto::CeresSolverOptions::MergeFrom(from.ceres_solver_options());
  }
  if (from.translation_weight() != 0) {
    set_translation_weight(from.translation_weight());
  }
  if (from.rotation_weight() != 0) {
    set_rotation_weight(from.rotation_weight());
  }
  if (from.only_optimize_yaw() != 0) {
    set_only_optimize_yaw(from.only_optimize_yaw());
  }
}

void CeresScanMatcherOptions3D::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CeresScanMatcherOptions3D::CopyFrom(const CeresScanMatcherOptions3D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CeresScanMatcherOptions3D::IsInitialized() const {
  return true;
}

void CeresScanMatcherOptions3D::Swap(CeresScanMatcherOptions3D* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CeresScanMatcherOptions3D::InternalSwap(CeresScanMatcherOptions3D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  occupied_space_weight_.InternalSwap(&other->occupied_space_weight_);
  swap(ceres_solver_options_, other->ceres_solver_options_);
  swap(translation_weight_, other->translation_weight_);
  swap(rotation_weight_, other->rotation_weight_);
  swap(only_optimize_yaw_, other->only_optimize_yaw_);
}

::google::protobuf::Metadata CeresScanMatcherOptions3D::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto);
  return ::file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f3d_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D* Arena::CreateMaybeMessage< ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
