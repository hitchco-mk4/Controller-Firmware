// 

#ifndef dataBlock_h
#define dataBlock_h

#include "Arduino.h"

#define BLOCKSIZE 64 // number of uint8_ts in a transmission block

union int_bytes {
  int i;
  byte bytes[2];
};

union float_bytes {
  float f;
  byte bytes[4];
};


class dataBlock {
  public:
    
    dataBlock();
    
    float db_f0;
    float db_f1;
    float db_f2;
    float db_f3;
    float db_f4;
    float db_f5;
    float db_f6;
    float db_f7;
    float db_f8;
    float db_f9;
    float db_f10;
    float db_f11;
  
    int db_i0;
    int db_i1;
    int db_i2;
    int db_i3;
  
    byte db_b0;
    byte db_b1;
    byte db_b2;
    byte db_b3;
    byte db_b4;
    byte db_b5;
    byte db_b6;

    void send_data_block(Stream &port);

  private:
    byte data_block[BLOCKSIZE];
};

#endif

