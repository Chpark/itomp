#ifndef DIFFERENTIATION_RULES_H_
#define DIFFERENTIATION_RULES_H_

namespace itomp_cio_planner
{

enum DIFF_RULE_NAMES
{
  DIFF_RULE_VELOCITY = 0, DIFF_RULE_ACCELERATION, DIFF_RULE_JERK, NUM_DIFF_RULES,
};

static const int DIFF_RULE_LENGTH = 7;
// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] =
{
    /*
     { -1 / 2.0, 0, 1 / 2.0 }, // velocity
     { 1 / 4.0, -1 / 2.0, 1 / 4.0 }, // acceleration
     { 0, 0, 0 } // jerk
     */
    /*
    { 0, 0, -2 / 6.0, -3 / 6.0, 6 / 6.0, -1 / 6.0, 0 }, // velocity
    { 0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0 }, // acceleration
    { 0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0 } // jerk
    */
    { 0, 0, -1 / 2.0, 0, 1 / 2.0, 0, 0 }, // velocity
    { 0, 0, 1 / 4.0, -1 / 2.0, 1 / 4.0, 0, 0 }, // acceleration
    { 0, 0, 0, 0, 0, 0, 0 } // jerk


};

}
#endif
