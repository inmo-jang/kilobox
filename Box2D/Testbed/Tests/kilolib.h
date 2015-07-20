// Copyright Simon Jones 2015

#ifndef KILOLIB_H
#define KILOLIB_H

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "freeglut/freeglut.h"
#endif


#include <random>
#include <queue>
#include <memory>
#include <iostream>
#include <string>
#include <cstdio>


#include "../Framework/Test.h"
#include "../Framework/Render.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f


typedef uint64_t usec_t;


// From http://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string string_format(const std::string& format, Args ... args){
    size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...);
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size);
}


class ModelStigmergy
{
public:
    struct colour_t { 
        GLfloat r; 
        GLfloat g; 
        GLfloat b; 
        GLfloat a;
        colour_t(GLfloat _r,GLfloat _g,GLfloat _b, GLfloat _a) : r(_r),g(_g),b(_b),a(_a){}
    };
};

struct Pose
{   
    float x;
    float y;
    float a;
};

class Color
{
public:
    float r,g,b,a;
	
    Color( float r, float g, float b, float a=1.0 ){}
	
    Color(){}
    
    // Set fraction in cubehelix palette
    Color(float f){}
	
    bool operator!=( const Color& other ) const;
    bool operator==( const Color& other ) const;
    static Color RandomColor();
    void Print( const char* prefix ) const;
    //void GLSet( void ) { glColor4f( r,g,b,a ); }
};

class CtrlArgs
{
public:
    std::string worldfile;
    std::string cmdline;
    
    CtrlArgs( std::string w, std::string c ) : worldfile(w), cmdline(c) {}
};

class ModelPosition
{
// Fake ModelPosition so that we can use the controller code as unchanged as possible from 
// the Stage controller
public:
    ModelPosition() {}

    b2World *world;
    Pose pose;
    Color color;
    Pose GetPose() 
    {
        return pose;
    }
    Pose GetGlobalPose()
    {
        return pose;
    }
    struct World
    {
        usec_t simtime;
        usec_t SimTimeNow() {return simtime;}
    } fake_world;
    void SetColor(Color col) {color = col;}
    World *GetWorld() {return &fake_world;}
    std::string token;
    const char *Token() {return token.c_str();}
};


namespace Kilolib
{

    // Forward declarations
    class Kilobot;

    // Subclass the built in contact class
    class KBContactListener : public b2ContactListener
    {
        void BeginContact(b2Contact *contact);
        void EndContact(b2Contact *contact);
    };


    enum entityCategory
    {
        KILOBOT         = 0x0001,
        MESSAGE         = 0x0002,
    };



    class Kilobot
    {
    public:
        static int ids;
        Kilobot(ModelPosition *_pos, Settings *_settings)
        :
            pos                 (_pos),
            m_world             (_pos->world),
            settings            (_settings),
            omega_goal          (0.0),
            xdot_goal           (0.0),
            ydot_goal           (0.0),
            ambient             (ModelStigmergy::colour_t(0,0,0,0)),
            led_r               (1.0),
            led_g               (1.0),
            led_b               (1.0),
            timer0_freq         (8e6/1024),
            master_tick_period  (32768),
            kilo_tx_period      (3906),
            message_period      (kilo_tx_period/timer0_freq),
            kilo_straight_left  (70),
            kilo_straight_right (70),
            kilo_turn_left      (70),
            kilo_turn_right     (70)
        {
            // Generate a unique ID, pre-increment so that first ID is 1
            kilo_uid = ++ids;
            make_kilobot(pos->GetPose().x, pos->GetPose().y, pos->GetPose().a);

            // Give the robot a name
            pos->token = string_format("kilobot:%d", kilo_uid);

            // Seed the random number generator with the unique ID
            gen.seed(settings->seed + kilo_uid);

            kilo_ticks_real = 0;//rand(0, 100);
            kilo_ticks      = kilo_ticks_real;
        }
        ModelPosition   *pos;
        b2Body          *m_body;
        b2World         *m_world;
        Settings        *settings;


        // Functions to maintain the list of other bots we are in range of
        void acquired(Kilobot *r)
        {
            //printf("adding   %d\n", r->kb_id);
            inrange_bots.push_back(r);
        }
        void lost(Kilobot *r)
        {
            //printf("removing %d\n", r->kb_id);
            inrange_bots.erase(std::find(inrange_bots.begin(), inrange_bots.end(), r));
        }




        void render();
        void update(float delta_t, float simtime);

    private:
        void make_kilobot(float xp, float yp, float th);
        void check_messages();
        
        float   dt;
        double  simtime;

        
        // Each kilobot has its own random number generator
        std::default_random_engine  gen;

        // Pointers to all kilobots within message range
        std::vector<Kilobot*>       inrange_bots;

        // Controller velocity goal, set by set_motor() based on two-wheel kinematics
        float                      omega_goal;
        float                      xdot_goal;
        float                      ydot_goal;
        // Pheromone strength
        float                      pheromone;
        ModelStigmergy::colour_t    ambient;

        // LED colour
        float led_r, led_g, led_b;
        
        


    protected:
        int   rand_intrange(int low, int high)
        {
            std::uniform_int_distribution<>   dist(low, high);
            return dist(gen);
        }
        float rand_gaussian(float sigma)
        {
            std::normal_distribution<float> dist(0.0, sigma);
            return dist(gen);
        }

        //------------------------------------------------------------
        // Kilobot API
        //------------------------------------------------------------
        
        // These methods are declared abstract virtual and must be defined
        // in the KBController class
        virtual void setup()    = 0;
        virtual void loop()     = 0;
        
        // This method will be called at the end of the simulation, overload
        // it to eg output stats
        virtual void finish() {
            //printf("%s\n",__PRETTY_FUNCTION__);
        }
        
        // Message container type
        typedef struct {
            uint8_t data[9]; ///< message payload.
            uint8_t type;    ///< message type.
            uint16_t crc;    ///< message crc.
        } message_t;

        // Distance measurement
        typedef struct {
            int16_t low_gain;
            int16_t high_gain;
        } distance_measurement_t;

        // Message queue - not actually part of the API
        struct m_event_t
        {
            message_t               m;
            distance_measurement_t  d;
            std::string             s;
        };
        typedef std::queue<m_event_t>   m_queue_t;
        m_queue_t                       message_queue;
      
        // Message stuff
        typedef enum {
            NORMAL = 0,
            GPS,
            BOOT = 0x80,
            BOOTPGM_PAGE,
            BOOTPGM_SIZE,
            RESET,
            SLEEP,
            WAKEUP,
            CHARGE,
            VOLTAGE,
            RUN,
            READUID,
            CALIB,
        } message_type_t;
        
        // Declared above so available for message event queue
//        // Message container type
//        typedef struct {
//            uint8_t data[9]; ///< message payload.
//            uint8_t type;    ///< message type.
//            uint16_t crc;    ///< message crc.
//        } message_t;
//        // Distance measurement
//        typedef struct {
//            int16_t low_gain;
//            int16_t high_gain;
//        } distance_measurement_t;
        
        // 16 bit CRC function from Atmel
        uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)
        {
            data ^= crc&0xff;
            data ^= data << 4;
            
            return ((((uint16_t)data << 8) | ((crc>>8)&0xff)) ^ (uint8_t)(data >> 4)
                    ^ ((uint16_t)data << 3));
        }
        uint16_t message_crc(const message_t *msg) {return 0;}
        
   
        distance_measurement_t  dist;
        message_t               msg;
        
        // Message system callback types
        typedef void        (Kilobot::*message_rx_t) (message_t *, distance_measurement_t *d);
        typedef message_t  *(Kilobot::*message_tx_t)(void);
        typedef void        (Kilobot::*message_tx_success_t)(void);
        // Dummy callbacks
        void                message_rx_dummy(message_t *m, distance_measurement_t *d) { }
        message_t          *message_tx_dummy() { return NULL; }
        void                message_tx_success_dummy() {}

        // Master tick period in usec, this is what Timer 0 is set to interrupt at
        double              timer0_freq;//         = 8e6/1024;
        const double        master_tick_period;//  = 32768;
        // Number of kilobot ticks since reset. Each tick is approximately every 30 ms,
        // so since the simulation tick defaults to 100ms, it will appear to increase in
        // chunks. This may affect the behaviour of some software..
        uint32_t            kilo_ticks;
        float               kilo_ticks_real;
        const uint32_t      kilo_tx_period;//      = 3906;
        const double        message_period;//      = kilo_tx_period / timer0_freq;
        uint32_t            last_message;
        // Unique ID for this kilobot
        uint16_t            kilo_uid;
        

        
        // Calibrated values for the motors
        int kilo_straight_left;//    = 50;
        int kilo_straight_right;//   = 50;
        int kilo_turn_left;//        = 40;
        int kilo_turn_right;//       = 40;
        
        // Message system callbacks
        message_rx_t            kilo_message_rx;
        message_tx_t            kilo_message_tx;
        message_tx_success_t    kilo_message_tx_success;
        
        //-------------------Not part of kilobot API------------------------------
        // Access to the message rx callback for senders
        //void                call_rx_callback(message_t *m, distance_measurement_t *d)
        //{
        //    (kilo_message_rx)(m,d);
        //}
        //------------------------------------------------------------------------

        // Received message and distance
        message_t               rx_msg;
        distance_measurement_t  rx_dist;
        
        uint8_t estimate_distance(const distance_measurement_t *dist)
        {
            return dist->low_gain;
        }
        
        uint8_t rand_hard()             {return rand_intrange(0,255);}
        uint8_t rand_soft()             {return rand_intrange(0,255);}

        void    rand_seed(uint16_t s)    {
            gen.seed(s);
        }
        int16_t get_ambientlight()
        {
            // ADC is 10 bits
            //
            // Return a low value for nest region, to the left,
            // a high value for the food region, to the right,
            // and zero for the wasteland in between
            float d = settings->kbnestfoodsep / 2.0;
            if (pos->pose.x < -d)
                return 100;
            if (pos->pose.x > d)
                return 900;
            return 0;
        }
        int16_t get_voltage()           {return 0;}
        int16_t get_temperature()       {return 0;}

        typedef ModelStigmergy::colour_t colour_t;
        //-------------------------------------------------
        colour_t get_ambient()
        {
            // THIS IS NOT PART OF THE STANDARD API!!
            // To be implemented using some sort of modulation
            // scheme on the ambient light sensing
            return ambient;
            //return colour_t(0,0,0,0);
        }
        void set_pheromone(double p) {pheromone = p;}

        //-------------------------------------------------


        void set_motors(int left_m, int right_m);


// We are not going to support delay, it needs nasty hacks like coroutines
// and the restriction the lack of it places on coding the kilobot is minimal.
// delay() is horrible anyway
//        void delay(int delay)
//        {
//            // Delay for approximately delay milliseconds
//            // Since the tick defaults to 100ms, this is necessarily very approximate.
//            // We ensure that there is at least one ticks delay, this means that code that
//            // does things like flash the led briefly does still make a visible difference
//            // on the GUI
//            delay_ticks = int(delay * 0.001 / dt);
//            if (delay_ticks < 1) delay_ticks = 1;
//            while (delay_ticks > 0)
//            {
//                delay_ticks--;
//                //printf("About to detach.. delay_ticks %d\n", delay_ticks);
//                Detach();
//            }
//            //printf("Resuming after delay..\n");
//        }
        
        void spinup_motors() {}

// Macro for easy use of set_color
#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)
        
        void set_color(uint8_t rgb)
        {
            led_r = (rgb&0x3)/3.0;
            led_g = ((rgb>>2)&0x3)/3.0;
            led_b = ((rgb>>4)&0x3)/3.0;
        }
      
        
    };

};



#endif
