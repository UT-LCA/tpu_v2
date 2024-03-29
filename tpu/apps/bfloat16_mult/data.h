
//In pixels
#define WIDTH 8
#define HEIGHT 8


char frame1[WIDTH*HEIGHT]={

  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0,
  0x00,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0

};

char frame2[WIDTH*HEIGHT]={

  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40,
  0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x40

};


//Using the following data for matmul just makes
//it a bit easier to debug. All 1s means the 
//output matrix should have all 8s
/*
char frame1[WIDTH*HEIGHT]={

  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01

};

char frame2[WIDTH*HEIGHT]={

  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01

};
*/

