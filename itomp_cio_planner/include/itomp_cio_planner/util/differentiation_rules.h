#ifndef DIFFERENTIATION_RULES_H_
#define DIFFERENTIATION_RULES_H_

namespace itomp_cio_planner
{

enum DIFF_RULE_NAMES
{
  DIFF_RULE_VELOCITY = 0, DIFF_RULE_ACCELERATION, DIFF_RULE_JERK, NUM_DIFF_RULES,
};

static const int DIFF_RULE_LENGTH = 3;
// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] =
{
    { -1 / 2.0, 0, 1 / 2.0 }, // velocity
    { 1 / 4.0, -1 / 2.0, 1 / 4.0 }, // acceleration
    { 0, 0, 0 } // jerk

};

}
#endif
