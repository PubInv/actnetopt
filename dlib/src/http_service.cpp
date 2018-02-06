/*
 * Example illustrating HTTP service.
 *
 * Server Usage:
 *    sudo ./distribution/example/http_service
 *
 * Client Usage:
 *    curl -w'\n' -v -X POST --data 'Hello, Restbed' 'http://localhost/resource'
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


void post_method_handler( const shared_ptr< Session > session )
{
    const auto request = session->get_request( );

    size_t content_length = request->get_header( "Content-Length", 0 );

    session->fetch( content_length, [ request ]( const shared_ptr< Session > session, const Bytes & body )
    {
        fprintf( stdout, "%.*s\n", ( int ) body.size( ), body.data( ) );
	// now what we really want to do here is to have constructed the playground and render
	// the dimensions. Making sure all of that linksl is an issue.

	// WARNING: This should really be in the physical space, not the viewport space!
	// Possibly body.size() must be checked here.
	auto j = json::parse(body.data(),body.data()+body.size());
	//	auto j = json::parse("{\"x\":4,\"y\":5}");	
	
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
	  for(int i = 0; i < an->num_nodes; i++) {
	    std::cout << " d["<< i << "]" << coordsx[i](0) << "," << coordsx[i](1) << std::endl;
	  }
	}


	
	fprintf( stdout, "%.*s\n", ( int ) body.size( ), body.data( ) );
	   
        session->close( OK, "Hello, World!", { { "Content-Length", "13" }, { "Connection", "close" } } );
    } );
}

int main( const int, const char** )
{
    auto resource = make_shared< Resource >( );
    resource->set_path( "/resource" );
    resource->set_method_handler( "POST", post_method_handler );

    // Here I set upa virtual space containing the TriLadder
    an = init_TriLadder();
  
    coordsx = new column_vector[an->num_nodes];

    mainx(an,coordsx,obstacle);


    Service service;
    service.publish( resource );
    service.start( );

    return EXIT_SUCCESS;
}
