
#include "debug_utils_c_version.h"
#include <rosidl_typesupport_introspection_c/field_types.h>
#include <stdio.h>
#include <stdlib.h>

void debug_parse_message(uint8_t* _data_buffer, const TypeInfo_t *_type_info)
{ 
  for (uint32_t i = 0; i < _type_info->member_count_; ++i) {
  
    // Get the introspection information for this particular member
    MemberInfo_t member_info = _type_info->members_[i];
    
    // Get a pointer to the member's data in the binary buffer
    uint8_t * member_data = &_data_buffer[member_info.offset_];

    // Recursively (because some members may be non-primitive types themselves) convert the member
    printf("Member[%u]: %s. Offset: %u \n", i, member_info.name_, member_info.offset_);
    debug_parse_member(member_info, member_data);
  }

}

/**
 * @function debug_parse_member
 */
void debug_parse_member(MemberInfo_t _member_info, uint8_t * _member_data)
{  
  // Print type of member (field in ROS2 message)
  const char* member_type_name = member_type_to_string(_member_info);
  printf( "\t * Member type: %s ", member_type_name);
  if (_member_info.is_array_) {
    // Only dealing with dynamic arrays for Phase I
    if (_member_info.is_upper_bound_ || _member_info.array_size_ == 0) {
      //dynamic_array_to_yaml( _member_info, _member_data);
    }
  } 
  
  else {
  
    if (_member_info.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {      
      debug_parse_message( (uint8_t *)(_member_data), (const TypeInfo_t *)(_member_info.members_->data) );
    } else {
      debug_parse_basic_value(_member_info, _member_data);
    }
    
  } // else is_array
  
}


/**
 * @brief Convert primitive value
 */
void debug_parse_basic_value(MemberInfo_t _member_info,
                             const uint8_t * _member_data)
{
  switch (_member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
    {  float val = *(const float *)(_member_data); 
       printf("\t\t Member[%s]: %f ", _member_info.name_, val);
    }
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
    {  double val = *(const double *)(_member_data); 
           printf("\t\t Member[%s]: %f ", _member_info.name_, val);    
    }
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
    {  long double val = *(const long double *)(_member_data); 
       printf("\t\t Member[%s]: %Lf ", _member_info.name_, val);    
    }
      break;
    /*case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
    {  const char* val = (const rosidl_runtime_c__String *)(_member_data)->data; 
           printf("\t\t Member[%s]: %s ", _member_info.name_, val);
    }
      break;*/
 
    default:
      break;
  }
  
}

// Convert primitive ROS types to a string representation
char* member_type_to_string(MemberInfo_t _member_info)
{
  char *result = malloc(sizeof(char)*100);
  switch (_member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      memcpy(result, "float", 5);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      memcpy(result, "buffer", 6);
      break;
    /*case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      result << "long double";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      result << "char";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      result << "wchar";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
      result << "boolean";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
      result << "octet";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      result << "uint8";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      result << "int8";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      result << "uint16";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      result << "int16";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      result << "uint32";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      result << "int32";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      result << "uint64";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      result << "int64";
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      result << "string";
      // Strings may have an upper bound
      if (_member_info.string_upper_bound_ > 0) {
        result << "<=" << _member_info.string_upper_bound_;
      }
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      result << "wstring";
      // WStrings may have an upper bound
      if (_member_info.string_upper_bound_ > 0) {
        result << "<=" << _member_info.string_upper_bound_;
      }
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
      // For nested types, the string representation must include the name space as well as the
      // type name
      result <<
        reinterpret_cast<const TypeInfo_t *>(_member_info.members_->data)->message_namespace_ <<
        "/" <<
        reinterpret_cast<const TypeInfo_t *>(_member_info.members_->data)->message_name_;
      break;*/
    default:
      // Don't throw an error, just print out "UNKNOWN" then keep persevering through the message
      //result << "UNKNOWN";
      break;
  }
  // If this member is a sequence of some kind, indicate that in the type
  /*if (_member_info.is_array_) {
    strncpy(result, "[", sizeof(result));
    if (_member_info.is_upper_bound_) {
      strncpy(result,"<=", sizeof(result));
    }
    if (_member_info.array_size_ > 0) {
      strncat( result, _member_info.array_size_, sizeof(buffer));
    } // HACK FOR NOW
    //strncat( result, "]", sizeof(result) );
  }*/
  return result;
}

// Get the size of primitive types
size_t size_of_member_type(uint8_t type_id)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      return sizeof(float);
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      return sizeof(double);
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      return sizeof(long double);
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
      return sizeof(bool);
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      return sizeof(int8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      return sizeof(int16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      return sizeof(uint32_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      return sizeof(int32_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      return sizeof(uint64_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      return sizeof(int64_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      return sizeof(rosidl_runtime_c__String);
    //case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
    //  return sizeof(rosidl_runtime_c__U16String);
   /* case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
      assert(0 && "Cannot get the size of a nested message");
      return 0;*/
    default:
      //assert(0 && "Cannot get the size of an unknown message type");
      return 0;
  }
}


//////////////////

// Convert the binary data for an individual element of an array member to YAML
void member_to_array_item(MemberInfo_t _member_info,
                          const uint8_t * _member_data)
{
  switch (_member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
    {
      float val = *(const float *)(_member_data);
      printf("\t\t *  %f ", val );
    }  
    break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
    {
      double val = *(const double *)(_member_data);
      printf("\t\t *  %f ", val );
    }  
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
    {
      long double val = *(const long double *)(_member_data);
      printf("\t\t *  %Lf ", val );
    }  
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
    {
      //array_node.push_back(
      //const char* val = (const rosidl_runtime_c__String *)(_member_data)->data;
      //printf("\t\t *  %s ", val );
    }
      break;

    default:
      //array_node.push_back("Unknown value for unknown type");
      break;
  }
}


// Convert a dynamically-sized sequence to YAML
/*void dynamic_array_to_yaml(const MemberInfo_t & member_info,
                      const uint8_t * member_data)
{
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__float__Sequence *>(member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__double__Sequence *>(member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__long_double__Sequence *>(member_data) );
      break;
 
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__String__Sequence *>(member_data));
      break;

    default:
      // Store an error string and keep going
      //array_node.push_back("Unknown value for unknown type");
      break;
  }
}

*/
