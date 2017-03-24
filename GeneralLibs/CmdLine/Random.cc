#include "Random.h"
#include "Allocator.h"
#include <set>
using namespace std;

namespace Torch {

// The initial seed.
unsigned long Random::the_initial_seed;

///// Code for the Mersenne Twister random generator....
const int Random::n = 624;
const int Random::m = 397;
int Random::left = 1;
int Random::initf = 0;
unsigned long *Random::next;
unsigned long Random::state[Random::n]; /* the array for the state vector  */
////////////////////////////////////////////////////////


/// For normal distribution
real Random::normal_x;
real Random::normal_y;
real Random::normal_rho;
bool Random::normal_is_valid = false;

void Random::seed()
{
  time_t ltime;
  struct tm *today;
  time(&ltime);
  today = localtime(&ltime);
  manualSeed((unsigned long)today->tm_sec);
}

///////////// The next 4 methods are taken from http://www.math.keio.ac.jp/matumoto/emt.html
///////////// Here is the copyright:
///////////// Some minor modifications have been made to adapt to "my" C++...

/*
   A C-program for MT19937, with initialization improved 2002/2/10.
   Coded by Takuji Nishimura and Makoto Matsumoto.
   This is a faster version by taking Shawn Cokus's optimization,
   Matthe Bellew's simplification, Isaku Wada's real version.

   Before using, initialize the state by using init_genrand(seed)
   or init_by_array(init_key, key_length).

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote
        products derived from this software without specific prior written
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   Any feedback is very welcome.
   http://www.math.keio.ac.jp/matumoto/emt.html
   email: matumoto@math.keio.ac.jp
*/

////////////////// Macros for the Mersenne Twister random generator...
/* Period parameters */  
//#define n 624
//#define m 397
#define MATRIX_A 0x9908b0dfUL   /* constant vector a */
#define UMASK 0x80000000UL /* most significant w-r bits */
#define LMASK 0x7fffffffUL /* least significant r bits */
#define MIXBITS(u,v) ( ((u) & UMASK) | ((v) & LMASK) )
#define TWIST(u,v) ((MIXBITS(u,v) >> 1) ^ ((v)&1UL ? MATRIX_A : 0UL))
/////////////////////////////////////////////////////////// That's it.

void Random::manualSeed(unsigned long the_seed_)
{
  the_initial_seed = the_seed_;
  state[0]= the_initial_seed & 0xffffffffUL;
  for(int j = 1; j < n; j++)
  {
    state[j] = (1812433253UL * (state[j-1] ^ (state[j-1] >> 30)) + j); 
    /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
    /* In the previous versions, mSBs of the seed affect   */
    /* only mSBs of the array state[].                        */
    /* 2002/01/09 modified by makoto matsumoto             */
    state[j] &= 0xffffffffUL;  /* for >32 bit machines */
  }
  left = 1;
  initf = 1;
}

unsigned long Random::getInitialSeed()
{
  if(initf == 0)
  {
    warning("Random: initializing the random generator");
    seed();
  }

  return the_initial_seed;
}

void Random::nextState()
{
  unsigned long *p=state;
  
  /* if init_genrand() has not been called, */
  /* a default initial seed is used         */
  if(initf == 0)
    seed();
//    manualSeed(5489UL);

  left = n;
  next = state;
    
  for(int j = n-m+1; --j; p++) 
    *p = p[m] ^ TWIST(p[0], p[1]);

  for(int j = m; --j; p++) 
    *p = p[m-n] ^ TWIST(p[0], p[1]);

  *p = p[m-n] ^ TWIST(p[0], state[0]);
}

unsigned long Random::random()
{
  unsigned long y;

  if (--left == 0)
    nextState();
  y = *next++;
  
  /* Tempering */
  y ^= (y >> 11);
  y ^= (y << 7) & 0x9d2c5680UL;
  y ^= (y << 15) & 0xefc60000UL;
  y ^= (y >> 18);

  return y;
}

/* generates a random number on [0,1)-real-interval */
real Random::uniform()
{
  unsigned long y;

  if(--left == 0)
    nextState();
  y = *next++;

  /* Tempering */
  y ^= (y >> 11);
  y ^= (y << 7) & 0x9d2c5680UL;
  y ^= (y << 15) & 0xefc60000UL;
  y ^= (y >> 18);
  
  return (real)y * (1.0/4294967296.0); 
  /* divided by 2^32 */
}

///
/// Thanks *a lot* Takuji Nishimura and Makoto Matsumoto!
///
/////////////////////////////////////////////////////////////////////
//// Now my own code...

void Random::getShuffledIndices(int *indices, int n_indices)
{
  for(int i = 0; i < n_indices; i++)
    indices[i] = i;
  
  Random::shuffle(indices, sizeof(int), n_indices);
}
	
void Random::shuffle(void *tabular, int size_elem, int n_elems)
{
  void *save = Allocator::sysAlloc(size_elem);
  char *tab = (char *)tabular;

  for(int i = 0; i < n_elems; i++)
  {
    int z = (int)(Random::uniform() * ((real)(n_elems-i-1)) );
    memcpy(save, tab+i*size_elem, size_elem);
    memcpy(tab+i*size_elem, tab+(z+i)*size_elem, size_elem);
    memcpy(tab+(z+i)*size_elem, save, size_elem);
  }
  free(save);
}

void Random::getRandomIndices(int *indices, int n_indices, int n, bool replacement)
{
	if ( n_indices > n )
		error("K (n_indices) must be less than or equal to N for sampling.");
	if ( replacement )
	{
		for ( int i = 0 ; i < n_indices ; i++ )
			indices[i] = (int)(Random::uniform() * (real)n );
	}
	else
	{
		if ( n_indices*4 > n )
		{
			int* tmp_indices = (int*)Allocator::sysAlloc(n*sizeof(int));
			Random::getShuffledIndices(tmp_indices, n);
			for ( int i = 0 ; i < n_indices ; i++ )
				indices[i] = tmp_indices[i];
			free((void*)tmp_indices);
		}
		else if ( n > n_indices*20 && n > 10 )
		{
			bool* flags = (bool*)Allocator::sysAlloc(n*sizeof(bool));
			for ( int i = 0 ; i < n ; i++ ) flags[i] = false;
			int sumf = 0;
			while ( sumf < n_indices ) {
				int loc = (int)(Random::uniform() * (real)n );
				if ( !flags[loc] )
				{
					indices[sumf] = loc;
					sumf++;
					flags[loc] = true;
				}
			}
			free((void*)flags);
		}
		else
		{
			std::set<int> rndset;
			std::set<int>::iterator it;
			while ( (int)rndset.size() < n_indices )
			{
				int loc = (int)(Random::uniform() * (real)n );
				it = rndset.find(loc);
				if ( it == rndset.end() ) rndset.insert(loc);
			}
			int* tmp_indices = indices;
			for (it=rndset.begin(); it!=rndset.end(); it++) (*tmp_indices++) = *it;
		}
	}
}
	
real Random::boundedUniform(real a, real b)
{
  return(Random::uniform() * (b - a) + a);
}

real Random::normal(real mean, real stdv)
{
  if(!normal_is_valid)
  {
    normal_x = Random::uniform();
    normal_y = Random::uniform();
    normal_rho = sqrt(-2. * log(1.0-normal_y));
    normal_is_valid = true;
  }
  else
    normal_is_valid = false;
  
  if(normal_is_valid)
    return normal_rho*cos(2.*M_PI*normal_x)*stdv+mean;
  else
    return normal_rho*sin(2.*M_PI*normal_x)*stdv+mean;
}

real Random::exponential(real lambda)
{
  return(-1. / lambda * log(1-Random::uniform()));
}

real Random::cauchy(real median, real sigma)
{
  return(median + sigma * tan(M_PI*(Random::uniform()-0.5)));
}

// Faut etre malade pour utiliser ca.
// M'enfin.
real Random::logNormal(real mean, real stdv)
{
  real zm = mean*mean;
  real zs = stdv*stdv;
  return(exp(Random::normal(log(zm/sqrt(zs + zm)), sqrt(log(zs/zm+1)) )));
}

int Random::geometric(real p)
{
  return((int)(log(1-Random::uniform()) / log(p)) + 1);
}

bool Random::bernouilli(real p)
{
  return(Random::uniform() <= p);
}

}
