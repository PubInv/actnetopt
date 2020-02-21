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
#include "posthole.hpp"

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
                    int debug = 1;
                    auto problem_json = request->get_query_parameter( "problem" );

                    cout << "PROBLEM" << problem_json << "\n";

                    auto j3 = json::parse(problem_json);

                    auto hj = j3["holes"];
                    auto pj = j3["posts"];
                    int n = hj["n"];

                    cout << "N = " << n << "\n";
                    auto hc = hj["C"];
                    auto pc = pj["C"];
                    auto ha = hj["A"];

                    cout << "hc = " << hc << "\n";

                    cout << "ha[0] = " << ha[0] << "\n";
                    cout << "hc[0][x] = " << hc[0]["x"] << "\n";

                    cout << "ha[0] = " << ha[0] << "\n";

                    Holes *h = new Holes();
                    h->n = n;
                    h->fixed_index = 0;
                    h->c = new column_vector[n];
                    h->a = new double[n];

                    Posts *p = new Posts();
                    p->n = n;
                    p->fixed_index = 0;
                    p->c = new column_vector[n];

                    cout << "made" << "\n";

                    for(int i = 0; i < n; i++) {
                      h->c[i].set_size(3);
                      h->c[i](0) = hc[i]["x"];
                      h->c[i](1) = hc[i]["y"];
                      h->c[i](2) = hc[i]["z"];
                      cout << i << "\n";
                      print_vec(h->c[i]);
                    }

                    for(int i = 0; i < n; i++) {
                      p->c[i].set_size(3);
                      p->c[i](0) = pc[i]["x"];
                      p->c[i](1) = pc[i]["y"];
                      p->c[i](2) = pc[i]["z"];
                      cout << i << "\n";
                      print_vec(p->c[i]);
                    }

                    PostHoleProblem *problem = new PostHoleProblem();
                    problem->h = h;
                    problem->p = p;

                    column_vector angles = problem->solve_problem();
                    double score = objective_f(angles);
                    if (debug) {
                      print_vec(angles);
                    }

                    if (true) {
                      //	    std::cout << " d["<< i << "]" << coordsx[i](0) << "," << coordsx[i](1) << "," << coordsx[i](2) << std::endl;

                      std::ostringstream ossx;
                      ossx << "{ \"score\": "<< score << ",\n"
                           << "\"angles\": [" << angles(0) << ", " << angles(1) << ", " << angles(2) << "]\n"
                           << "}\n";
                      std::string s = ossx.str();
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
