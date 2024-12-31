#include <edoras_core/interface.h>
#include <stdio.h>
#include <string.h>

/***********************************/
int main(int argc, char* argv[])
{
   const TypeInfo_t * ti; 
   void* ts_library;
   const TypeSupport_t* ts;
   const char* interface_type = "geometry_msgs";
   const char* interface_name = "Pose";
   ti = get_type_info(interface_type, interface_name);
   printf("**M** Type info for geometry_msgs::Pose has %u members \n", ti->member_count_);
   printf("**M** Loading type support...\n");
   ts_library = get_type_support_library(interface_type, interface_name);
   ts = get_type_support(interface_type, interface_name, ts_library);
   const char* tpid = ts->typesupport_identifier;
   printf("**M** Type support identifier in gateway_app: %s \n", tpid);
        
   // Parse the information  
   size_t offset = 8;
   size_t sixty = 60;
        
   size_t offset_full = offset + 2*sizeof(size_t);     
   uint8_t data[sixty + offset_full];
   for(size_t i = 0; i < sixty + offset_full; ++i)
     data[i] = 0x00;

  memcpy(data + offset, &sixty, sizeof(size_t));
  memcpy(data + offset + sizeof(size_t), &sixty, sizeof(size_t));   

  data[0] = 0x18;
  data[1] = 0x27;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00; 
  data[5] = 0x4d; 
  data[6] = 0x01; 
  data[7] = 0x00;

   	
   data[1+ offset_full] = 0x01;   
   data[10 + offset_full] = 0x37; 
   data[11 + offset_full] = 0x40;
   data[18 + offset_full] = 0x3d; 
   data[19 + offset_full] = 0x40; 
   data[26 + offset_full] = 0x28; 
   data[27 + offset_full] = 0x40; 
   data[57 + offset_full] = 0x04; 
   data[58 + offset_full] = 0x9f; 
   data[59 + offset_full] = 0x40;

   uint8_t* di = data; 

   printf("Data: ");
   for(size_t i = 0; i < sixty + offset_full; ++i)
     printf(" %02x ", data[i]);
   printf("\n");

   
   size_t buffer_length_test;
   memcpy(&buffer_length_test, di + offset, sizeof(size_t));
   printf("Reading raw: buffer length test: %zu (%lu bytes) \n", buffer_length_test, sizeof(size_t));
   uint8_t* msg_pointer;
   offset = 8;
   size_t buffer_size;
   msg_pointer = from_uint_buffer_to_msg_pointer( di, offset, ts, ti, &buffer_size);
   printf("Size of msg pointer: %lu \n", sizeof(msg_pointer));
   printf("Buffer size: %ld\n", buffer_size);
   ///debug_parse_buffer(msg_pointer, ti);
   
}   
