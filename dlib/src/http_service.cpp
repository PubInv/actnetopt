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
#include "TriLadder.hpp"
#include "Obstacle.hpp"
#include "playground.hpp"
#include <dlib/optimization.h>

#include "custom_logger.hpp"

using namespace std;
using namespace restbed;



// for convenience
using json = nlohmann::json;

TriLadder *an;
  
column_vector* coordsx;


void get_method_handler_goal( const shared_ptr< Session > session )
{
    const auto request = session->get_request( );

    size_t content_length = request->get_header( "Content-Length", 0 );

    session->fetch( content_length, [ request ]( const shared_ptr< Session > session, const Bytes & body )
    {

      cout << "XXX" << request->get_query_parameter( "x" ) << "\n";
      cout << "YYY" << request->get_query_parameter( "y" ) << "\n";
      
      double x = request->get_query_parameter( "x",10.0 );
      double y = request->get_query_parameter( "y",10.0 );
      
      if (true) {
	  // there is an entry with key "foo"
	  cout << "FOUND X AND Y!\n";
	  handle_goal_target_physical(an,coordsx,x,y);
	  json solution;
	  for(int i = 0; i < an->num_nodes; i++) {
	    std::cout << " d["<< i << "]" << coordsx[i](0) << "," << coordsx[i](1) << std::endl;
	    solution[i]["x"] = coordsx[i](0);
	    solution[i]["y"] = coordsx[i](1);	    
	  }

	  std::string s = solution.dump(4);
	  fprintf( stdout, "%s\n", s.c_str() );
	  std::ostringstream oss;
	  oss << s.length();
	  std::string len = oss.str();
	  session->close( OK, s.c_str(), {  { "Content-Length", len } } );	  
	} else {
	  session->close( BAD_REQUEST, "could not find x or y", { { "Content-Length", "21" }, { "Connection", "close" } } );	  
	}
	
	//	fprintf( stdout, "%.*s\n", ( int ) body.size( ), body.data( ) );
	// output the solution as JSON!

    } );
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
    resource->set_path( "/goal" );
    resource->set_method_handler( "GET", get_method_handler_goal );
    // This is to deal with the CORS stuff by sending back "Access-Control-Allow-Headers"
    resource->set_method_handler( "OPTIONS", options_method_handler_goal );

    auto settings = make_shared< Settings >( );
    //    settings->set_port( 1984 );
    //    settings->set_default_header( "Connection", "close" );
    settings->set_default_header( "Access-Control-Allow-Origin", "*" );    

    

    // Here I set upa virtual space containing the TriLadder
    // This could be moved into a different call to allow
    // the truss parameters to be set.
    an = init_TriLadder();
  
    coordsx = new column_vector[an->num_nodes];

    mainx(an,coordsx,obstacle);


    Service service;
    service.publish( resource );
    service.set_logger( make_shared< CustomLogger >( ) );
    service.start(settings );

    return EXIT_SUCCESS;
}
