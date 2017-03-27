//Include ARIA libraries (S)
#include "Aria.h"
#include "ArLaser.h"
#include "ArNetworking.h"
#include "ArServerModeJogPosition.h"
#include "ArVideo.h"
#include "ArVideoConnector.h"
#include "ArPTZConnector.h"
//Include ARIA libraries (E)

#include <stdexcept>
#include <string>
#include <iostream>

class Server
{
public:
    Server( ArRobot* robot );
    void callback_ModeJogPosition_active(void);

private:
    ArRobot* my_robot;
};

Server::Server( ArRobot* robot ) :
    my_robot( robot )
{
    return;
}

void Server::callback_ModeJogPosition_active()
{
    std::cout<<"Rozpoczeta przemieszczanie\n";
}

int main(int argc, char **argv)
{
//    Initialize main Aria objects: Aria and ArVideo
    Aria::init();
    ArVideo::init();

//    Turn off normal Aria logging
    ArLog::setLogLevel( ArLog::Terse );

//    Manage passed parameters and specify camera on Pioneer device (hardcoded)
    ArArgumentParser parser( &argc, argv);
    parser.addDefaultArgument("-ptzType vcc50i -videoType pxc");
    parser.loadDefaultArguments();

//    Robot device object initialization
    ArRobot robot;
    ArRobotConnector robotConnector( &parser, &robot );

//    Camera management objects from ArVideo library init (1)
    ArPTZConnector server_ArPTZConnector( &parser, &robot );
    ArVideoConnector server_ArVideoConnector( &parser, &robot );

    try
    {
//        Try to connect Pioneer microcontroller
        if ( !robotConnector.connectRobot() )
            throw std::runtime_error( std::string("Could not connect to robot.") );

//        If connection to the robot is successfull, then run the robot
//        in asynchronus mod.
        robot.runAsync( true );

        if ( !Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed() )
            throw std::runtime_error( std::string("Parsing problem." ) );
    }
    catch( std::exception &e)
    {
        printf("Robot initialization error: %s\n", e.what() );
        Aria::exit();
    }

//    Server initialization
    ArServerBase server;
    ArServerSimpleOpener serverOpener( &parser );

//    Server mods / settings activation (S)
    ArServerInfoRobot server_ArServerInfoRobot( &server, &robot );
    ArServerInfoSensor server_ArServerInfoSensor( &server, &robot );
    ArServerInfoStrings server_ArServerInfoStrings( &server );

//    Mods for steering
    ArServerModeRatioDrive server_ArServerModeRatioDrive( &server, &robot );
    ArServerModeStop server_ArServerModeStop( &server, &robot );
    ArServerModeJogPosition server_ArServerModeJogPosition( &server, &robot );

    server_ArServerModeJogPosition.addToConfig(Aria::getConfig());
    server_ArServerModeStop.addAsDefaultMode();
    server_ArServerModeStop.activate();
//    Following Server object might be removed in future
    Server serverOperator( &robot );
    ArFunctorC<Server> functor_callback_ModeJogPosition_active( serverOperator,
            &Server::callback_ModeJogPosition_active);

    server_ArServerModeJogPosition.addActivateCallback( &functor_callback_ModeJogPosition_active );
//    Server mods / settings activation (E)

//    Camera management objects from ArVideo library init (2)
//    There is a problem with MobileSim regarding usage of cameras.
//    At the moment it is possible to test camera code only with real robot
//    device thus it is wise to comment below code out in order to work
//    with MobileSim.
    try
    {
        if ( !server_ArVideoConnector.connect() )
            throw std::runtime_error( std::string("Could not connect to video devices."));
        if (server_ArVideoConnector.getNumFrameGrabbers() == 0 )
            throw std::runtime_error( std::string("Could not find any video device."));

        if (!server_ArPTZConnector.connect())
            throw std::runtime_error( std::string("Could not connect to PTZ controls."));
        if (server_ArPTZConnector.getNumPTZs() == 0)
            throw std::runtime_error( std::string("Could not find any PTZ control device."));

        if(!ArVideo::createServers(&server, &server_ArVideoConnector, &server_ArPTZConnector))
            throw std::runtime_error( std::string("Could not create Video server."));
    }
    catch( std::exception &e)
    {
        printf("Video server error: %s\n", e.what() );
        Aria::exit();
    }

    try
    {
        if ( !serverOpener.open( &server ) )
            throw std::runtime_error( std::string("Could not open server on port: %d",
                                                  serverOpener.getPort() ) );

        // Start Server
        server.runAsync();
    }
    catch( std::exception &e)
    {
        printf("Server initialization error: %s\n", e.what() );
        Aria::exit();
    }

//    Add laser server (S)
    ArLaserConnector server_ArLaserConnector( &parser, &robot, &robotConnector);
    std::map<int, ArLaser*>* lasers_map;

    try
    {
        if( !server_ArLaserConnector.connectLasers() )
            throw std::runtime_error( std::string("Could not connect laser.") );

        lasers_map = robot.getLaserMap();
        if( lasers_map->size() < 1)
            throw std::runtime_error( std::string("Could not find any laser.") );

//        Laser testing
//        server_Laser = (*lasers_map)[1];
//        printf("Laser range: %d\n", server_Laser->getAbsoluteMaxRange());
//        fflush(stdout);
    }
    catch( std::exception &e)
    {
        printf("Laser initialization error: %s", e.what() );
        Aria::exit();
    }
//    Add laser server (E)

//    Turn Pioneer's motors on
    robot.lock();
    robot.enableMotors();
    robot.unlock();

//    Main program loop
    while(true)
    {
        ArUtil::sleep( 100 );
    }
    Aria::exit(0);
}
