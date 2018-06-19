#ifndef data_block_h
#define data_block_h

#define BLOCKSIZE 64 // number of bytes in a transmission block
byte data_block[BLOCKSIZE] = { 0 }; 

union int_bytes {
  int i;
  byte bytes[2];
};

union float_bytes {
  float f;
  byte bytes[4];
};

float db_f0 = 0;
float db_f1 = 0;
float db_f2 = 0;
float db_f3 = 0;
float db_f4 = 0;
float db_f5 = 0;
float db_f6 = 0;
float db_f7 = 0; 
float db_f8 = 0;
float db_f9 = 0;
float db_f10 = 0;
float db_f11 = 0;

int db_i0 = 0;
int db_i1 = 0;
int db_i2 = 0;
int db_i3 = 0;

byte db_b0 = 0;
byte db_b1 = 0;
byte db_b2 = 0;
byte db_b3 = 0;
byte db_b4 = 0;
byte db_b5 = 0;
byte db_b6 = 0;

void pack_data_block(void) {

  float_bytes f0;
  float_bytes f1;
  float_bytes f2;
  float_bytes f3;
  float_bytes f4;
  float_bytes f5;
  float_bytes f6;
  float_bytes f7;
  float_bytes f8;
  float_bytes f9;
  float_bytes f10;
  float_bytes f11;

  f0.f = db_f0;
  f1.f = db_f1;
  f2.f = db_f2;
  f3.f = db_f3;
  f4.f = db_f4;
  f5.f = db_f5;
  f6.f = db_f6;
  f7.f = db_f7;
  f8.f = db_f8;
  f9.f = db_f9;
  f10.f = db_f10;
  f11.f = db_f11;

  data_block[0] = f0.bytes[0];
  data_block[1] = f0.bytes[1];
  data_block[2] = f0.bytes[2];
  data_block[3] = f0.bytes[3];
  data_block[4] = f1.bytes[0];
  data_block[5] = f1.bytes[1];
  data_block[6] = f1.bytes[2];
  data_block[7] = f1.bytes[3];
  data_block[8] = f2.bytes[0];
  data_block[9] = f2.bytes[1];
  data_block[10] = f2.bytes[2];
  data_block[11] = f2.bytes[3];
  data_block[12] = f3.bytes[0];
  data_block[13] = f3.bytes[1];
  data_block[14] = f3.bytes[2];
  data_block[15] = f3.bytes[3];
  data_block[16] = f4.bytes[0];
  data_block[17] = f4.bytes[1];
  data_block[18] = f4.bytes[2];
  data_block[19] = f4.bytes[3];
  data_block[20] = f5.bytes[0];
  data_block[21] = f5.bytes[1];
  data_block[22] = f5.bytes[2];
  data_block[23] = f5.bytes[3];
  data_block[24] = f6.bytes[0];
  data_block[25] = f6.bytes[1];
  data_block[26] = f6.bytes[2];
  data_block[27] = f6.bytes[3];
  data_block[28] = f7.bytes[0];
  data_block[29] = f7.bytes[1];
  data_block[30] = f7.bytes[2];
  data_block[31] = f7.bytes[3];
  data_block[32] = f8.bytes[0];
  data_block[33] = f8.bytes[1];
  data_block[34] = f8.bytes[2];
  data_block[35] = f8.bytes[3];
  data_block[36] = f9.bytes[0];
  data_block[37] = f9.bytes[1];
  data_block[38] = f9.bytes[2];
  data_block[39] = f9.bytes[3];
  data_block[40] = f10.bytes[0];
  data_block[41] = f10.bytes[1];
  data_block[42] = f10.bytes[2];
  data_block[43] = f10.bytes[3];
  data_block[44] = f11.bytes[0];
  data_block[45] = f11.bytes[1];
  data_block[46] = f11.bytes[2];
  data_block[47] = f11.bytes[3];

  int_bytes i0;
  int_bytes i1;
  int_bytes i2;
  int_bytes i3;

  i0.i = db_i0;
  i1.i = db_i1;
  i2.i = db_i2;
  i3.i = db_i3;

  data_block[48] = i0.bytes[0];
  data_block[49] = i0.bytes[1];

  data_block[50] = i1.bytes[0];
  data_block[51] = i1.bytes[1];

  data_block[52] = i2.bytes[0];
  data_block[53] = i2.bytes[1];

  data_block[54] = i3.bytes[0];
  data_block[55] = i3.bytes[1];

  data_block[56] = db_b0;
  data_block[57] = db_b1;
  data_block[58] = db_b2;
  data_block[59] = db_b3;
  data_block[60] = db_b4;
  data_block[61] = db_b5;
  data_block[62] = db_b6;

  byte crc;
  
  for (int index = 0; index < BLOCKSIZE - 1; index++) {
    crc = crc + data_block[index];
  }
  
  data_block[63] = crc;
  
}

#endif

