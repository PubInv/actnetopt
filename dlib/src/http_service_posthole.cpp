/*
 * Note: This is now modified to access the actnetopt optimizer.
 * Copyright (C) Robert L. Read, 2018, license MIT or GPL3 (not sure which!)
 * Example illustrating HTTP service.
 *
 * Server Usage:
 *    sudo ./distribution/example/http_service
 *
 * Client Usage:
 *    curl -w'\n' -v -X POST --data 'Hello, Restbed' 'http://localhost/goal'
 */

#include <memory>
#include <cstdlib>
#include <restbed>
#include "json.hpp"

// #include "Obstacle.hpp"
#include <dlib/optimization.h>
#include "custom_logger.hpp"
#include "ActNetUtility.hpp"

using namespace std;
using namespace restbed;



// for convenience
using json = nlohmann::json;

column_vector* coordsx;


void get_method_handler_goal( const shared_ptr< Session > session )
{
    cerr << "goal invoked\n";

    const auto request = session->get_request( );

    size_t content_length = request->get_header( "Content-Length", 0 );

    session->fetch( content_length, [ request ]( const shared_ptr< Session > session, const Bytes & body )
    {

      auto problem = request->get_query_parameter( "problem" );

      cout << "PROBLEM" << problem << "\n";

      auto j3 = json::parse(problem);

      auto h = j3["holes"];
      auto p = j3["posts"];
      int n = h["n"];

      cout << "N = " << n << "\n";
      auto hc = h["C"];
      auto ha = h["A"];

      cout << "hc = " << hc << "\n";

      cout << "ha[0] = " << ha[0] << "\n";
      cout << "hc[0][x] = " << hc[0]["x"] << "\n";

      cout << "ha[0] = " << ha[0] << "\n";

      int i = 0;
      for (json::iterator it = hc.begin(); it != hc.end(); ++it) {
	std::cout << "hc" << i++ << "\n";
	std::cout << it.key() << " : " << it.value() << "\n";
	auto v = it.value();
      }
      for (json::iterator it = ha.begin(); it != ha.end(); ++it) {
	std::cout << "ha" << i++ << "\n";
	std::cout << it.key() << " : " << it.value() << "\n";
	auto v = it.value();
      }

      if (true) {
        //	    std::cout << " d["<< i << "]" << coordsx[i](0) << "," << coordsx[i](1) << "," << coordsx[i](2) << std::endl;
            std::string s = "Euler 0,1,2";
            fprintf( stdout, "%s\n", s.c_str() );
            std::ostringstream oss;
            oss << s.length();
            std::string len = oss.str();
            session->close( OK, s.c_str(), {  { "Content-Length", len } } );
      } else {
        session->close( BAD_REQUEST, "could not find x or y", { { "Content-Length", "21" }, { "Connection", "close" } } );
      }
    });
}

void options_method_handler_goal( const shared_ptr< Session > session )
{
    const auto request = session->get_request( );
    session->close( OK, "SUCCESS", {
        {"Access-Control-Allow-Headers", "*"
            } } );
}

int main( const int, const char** )
{
    auto resource = make_shared< Resource >( );
    resource->set_path( "/posthole" );
    resource->set_method_handler( "GET", get_method_handler_goal );
    // This is to deal with the CORS stuff by sending back "Access-Control-Allow-Headers"
    resource->set_method_handler( "OPTIONS", options_method_handler_goal );

    auto settings = make_shared< Settings >( );
    //    settings->set_port( 1984 );
    //    settings->set_default_header( "Connection", "close" );
    settings->set_default_header( "Access-Control-Allow-Origin", "*" );

    Service service;
    service.publish( resource );
    service.set_logger( make_shared< CustomLogger >( ) );
    service.start(settings );
    cerr << "Service Invoked!\n";

    return EXIT_SUCCESS;
}
