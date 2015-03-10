//
//  DebugLogger.h
//  BeatSeekerOF
//
//  Created by Andrew Robertson on 02/10/2014.
//
//

#ifndef _DebugLogger_h
#define _DebugLogger_h

#include <iostream>

//in C flags , add -DNDEBUG, -DKDEGUB, -DHDEBUG to suppress appropriate Debud std::streambuf


//using namespace std;
class NoDebug
{
public:
    template <typename T>
    NoDebug &operator<<(const T &) {
        return *this;
    }
   
    NoDebug &operator<<(std::ostream &(*)(std::ostream &)) {
        return *this;
    }
   

};

#ifndef NDEBUG
#define DSTREAM std::cerr
#else
#define DSTREAM (NoDebug())
#endif


#ifndef HDEBUG
#define HSTREAM std::cerr
#else
#define HSTREAM (NoDebug())
#endif

#ifndef KDEBUG
#define KSTREAM std::cerr
#else
#define KSTREAM (NoDebug())
#endif


#ifndef PDEBUG
#define PSTREAM std::cerr
#else
#define PSTREAM (NoDebug())
#endif


#endif




/*
 // Use a typedef to make the code readable.
 // This is a function pointer that takes a stream as input and returns the stream.
 // This covers functions like std::endl
 typedef std::ostream& (*STRFUNC)(std::ostream&);
 
 NoDebug &operator<<(STRFUNC func)  // Inside the class
 {
 // Apply the function
 //func(std::cerr);
 // But return the debug object
 return *this;
 }
 */