#ifndef LISA_TEST_BOARDS
#define LISA_TEST_BOARDS

enum TestType {
  TestTypeNone,
  TestTypeBaro,
  TestTypeBldc,
  TestTypeSrvo,
  TestTypeUARTS
};

extern enum TestType cur_test;

#define test_board_SetCurTest(_val) { \
    cur_test = _val;		      \
    start_test();		      \
}

extern void start_test(void);

#endif /* LISA_TEST_BOARDS */
