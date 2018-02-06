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

using namespace std;
using namespace restbed;



// for convenience
using json = nlohmann::json;

TriLadder *an;
  
column_vector* coordsx;


void post_method_handler_goal( const shared_ptr< Session > session )
{
    const auto request = session->get_request( );

    size_t content_length = request->get_header( "Content-Length", 0 );

    session->fetch( content_length, [ request ]( const shared_ptr< Session > session, const Bytes & body )
    {
	auto j = json::parse(body.data(),body.data()+body.size());
	
	cout << "SAMPLE JSON\n";
	cout << j << "\n";

	// range-based for
	for (auto& element : j) {
	  cout << "Element\n";
	  std::cout << element << '\n';
	}
	
// find an entry
	if ((j.find("x") != j.end()) && (j.find("y") != j.end())) {
	  // there is an entry with key "foo"
	  cout << "FOUND X AND Y!\n";
	  cout << j["x"] << "\n";
	  cout << j["y"] << "\n";
	  handle_goal_target_physical(an,coordsx,j["x"],j["y"]);
	  // Need to output as JSON here....
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

int main( const int, const char** )
{
    auto resource = make_shared< Resource >( );
    resource->set_path( "/goal" );
    resource->set_method_handler( "GET", post_method_handler_goal );

    // Here I set upa virtual space containing the TriLadder
    // This could be moved into a different call to allow
    // the truss parameters to be set.
    an = init_TriLadder();
  
    coordsx = new column_vector[an->num_nodes];

    mainx(an,coordsx,obstacle);


    Service service;
    service.publish( resource );
    service.start( );

    return EXIT_SUCCESS;
}
