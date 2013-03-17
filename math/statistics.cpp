#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <sys/math/statistics.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        template<typename Scalar>
        class RandomNumberGenerator {
            private:
                boost::mt19937 rng;
                boost::normal_distribution<Scalar> norm;
                boost::variate_generator<boost::mt19937&,boost::normal_distribution<Scalar> > gen;
            public:
                typedef Scalar result_type;
                RandomNumberGenerator() : gen(rng,norm) {}
                inline const Scalar operator()() { return gen(); }
        };

        template<typename T> T randN() {
            static RandomNumberGenerator<T> generator;
            return generator();
        }

        template F32 randN<F32>();
        template F64 randN<F64>();
    }
}
