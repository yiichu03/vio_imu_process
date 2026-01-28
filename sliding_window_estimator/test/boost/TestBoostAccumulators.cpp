
#include <gtest/gtest.h>

#include <iostream>
#include <random>
#include <vector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


TEST(Boost, Accumulators)
{
    using namespace std;
    using namespace boost::accumulators;

    // Define an accumulator set for calculating the mean and the
    // 2nd moment ...

    boost::accumulators::accumulator_set<
          double,
          boost::accumulators::features<
        boost::accumulators::tag::sum,
        boost::accumulators::tag::min,
        boost::accumulators::tag::max,
        boost::accumulators::tag::rolling_mean,
        boost::accumulators::tag::mean
        >
          > acc(boost::accumulators::tag::rolling_window::window_size = 7);

    // push in some data ...
    acc(1.2);
    acc(2.3);
    acc(3.4);
    acc(4.5);

    // Display the results ...
    ASSERT_EQ(boost::accumulators::mean(acc), 2.85);
}


typedef boost::iterator_range<
    std::vector<std::pair<double, double>>::iterator>
    HistogramType;


HistogramType func() {
  boost::accumulators::accumulator_set<
      double, boost::accumulators::features<boost::accumulators::tag::count,
                                            boost::accumulators::tag::density>>
      tally(boost::accumulators::tag::density::num_bins = 20,
            boost::accumulators::tag::density::cache_size = 40);
  const int nrolls = 10000; // number of experiments

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 2.0);

  for (int i = 0; i < nrolls; ++i) {
    double number = distribution(generator);
    tally(number);
  }

  HistogramType hist = boost::accumulators::density(tally);
  return hist;
}

TEST(Boost, Histogram) {
  HistogramType hist = func();
  double total = 0.0;
  for (size_t i = 0; i < hist.size(); i++) {
    total += hist[i].second;
  }
  EXPECT_LT(std::fabs(total - 1.0), 1e-2) << "Total of feature histogram densities " << total << " != 1.\n";
}
