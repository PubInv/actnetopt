//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>


#include <math.h>

#define PI 3.14159265

class MyTest {
public:
  MyTest(char* s) {
  }
  bool is_valid() {
    return true;
  }
};

#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( my_test )
{
    MyTest test_object( "qwerty" );

    std::cout << "YES, THE TEST FILE RAN!";

    BOOST_CHECK( test_object.is_valid() );
}
