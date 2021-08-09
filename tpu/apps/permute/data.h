
//In pixels
#define WIDTH 16
#define HEIGHT 8


char frame1[WIDTH*HEIGHT]={
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
  0x00,0x07,0x00,0x06,0x00,0x05,0x00,0x04,0x00,0x03,0x00,0x02,0x00,0x01,0x00,0x00,
};

char frame2[WIDTH*HEIGHT]={
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
  0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,
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

