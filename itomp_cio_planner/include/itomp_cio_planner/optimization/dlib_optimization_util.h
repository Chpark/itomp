#ifndef DLIB_OPTIMIZATION_UTIL_H_
#define DLIB_OPTIMIZATION_UTIL_H_

#include <cmath>
#include <limits>
#include "dlib/optimization/optimization_abstract.h"
#include "dlib/optimization/optimization_search_strategies.h"
#include "dlib/optimization/optimization_stop_strategies.h"
#include "dlib/optimization/optimization_line_search.h"
#include "dlib/optimization/optimization.h"
#include <itomp_cio_planner/util/jacobian.h>

namespace itomp_cio_planner
{
class lbfgs_search_strategy
{
public:
    explicit lbfgs_search_strategy(unsigned long max_size_) : max_size(max_size_), been_used(false)
    {
        DLIB_ASSERT (
            max_size > 0,
            "\t lbfgs_search_strategy(max_size)"
            << "\n\t max_size can't be zero"
        );
    }

    lbfgs_search_strategy(const lbfgs_search_strategy& item)
    {
        max_size = item.max_size;
        been_used = item.been_used;
        prev_x = item.prev_x;
        prev_derivative = item.prev_derivative;
        prev_direction = item.prev_direction;
        alpha = item.alpha;
        dh_temp = item.dh_temp;
    }

    double get_wolfe_rho (
    ) const { return 0.01; }

    double get_wolfe_sigma (
    ) const { return 0.9; }

    unsigned long get_max_line_search_iterations (
    ) const { return 14; }

    template <typename T>
    const dlib::matrix<double,0,1>& get_next_direction (
        const T& x,
        const double ,
        const T& funct_derivative
    )
    {
        prev_direction = -funct_derivative;

        if (been_used == false)
        {
            been_used = true;
        }
        else
        {
            // add an element into the stored data sequence
            dh_temp.s = x - prev_x;
            dh_temp.y = funct_derivative - prev_derivative;
            double temp = dot(dh_temp.s, dh_temp.y);
            // only accept this bit of data if temp isn't zero
            if (std::abs(temp) > std::numeric_limits<double>::epsilon())
            {
                dh_temp.rho = 1/temp;
                data.add(data.size(), dh_temp);
            }
            else
            {
                data.clear();
            }

            if (data.size() > 0)
            {
                // This block of code is from algorithm 7.4 in the Nocedal book.

                alpha.resize(data.size());
                for (unsigned long i = data.size()-1; i < data.size(); --i)
                {
                    alpha[i] = data[i].rho*dot(data[i].s, prev_direction);
                    prev_direction -= alpha[i]*data[i].y;
                }

                // Take a guess at what the first H matrix should be.  This formula below is what is suggested
                // in the book Numerical Optimization by Nocedal and Wright in the chapter on Large Scale
                // Unconstrained Optimization (in the L-BFGS section).
                double H_0 = 1.0/data[data.size()-1].rho/dot(data[data.size()-1].y, data[data.size()-1].y);
                H_0 = dlib::put_in_range(0.001, 1000.0, H_0);
                prev_direction *= H_0;

                for (unsigned long i = 0; i < data.size(); ++i)
                {
                    double beta = data[i].rho*dot(data[i].y, prev_direction);
                    prev_direction += data[i].s * (alpha[i] - beta);
                }
            }

        }

        if (data.size() > max_size)
        {
            // remove the oldest element in the data sequence
            data.remove(0, dh_temp);
        }

        prev_x = x;
        prev_derivative = funct_derivative;
        return prev_direction;
    }

private:

    struct data_helper
    {
        dlib::matrix<double,0,1> s;
        dlib::matrix<double,0,1> y;
        double rho;

        friend void swap(data_helper& a, data_helper& b)
        {
            a.s.swap(b.s);
            a.y.swap(b.y);
            std::swap(a.rho, b.rho);
        }
    };
    dlib::sequence<data_helper>::kernel_2a data;

    unsigned long max_size;
    bool been_used;
    dlib::matrix<double,0,1> prev_x;
    dlib::matrix<double,0,1> prev_derivative;
    dlib::matrix<double,0,1> prev_direction;
    std::vector<double> alpha;

    data_helper dh_temp;
};
// ----------------------------------------------------------------------------------------
//                    Functions that perform unconstrained optimization
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    template <
        typename search_strategy_type,
        typename stop_strategy_type,
        typename funct,
        typename funct_der,
        typename T
        >
    double find_min (
        search_strategy_type search_strategy,
        stop_strategy_type stop_strategy,
        const funct& f,
        const funct_der& der,
        T& x,
        double min_f
    )
    {
        COMPILE_TIME_ASSERT(dlib::is_matrix<T>::value);
        // The starting point (i.e. x) must be a column vector.
        COMPILE_TIME_ASSERT(T::NC <= 1);

        DLIB_CASSERT (
            is_col_vector(x),
            "\tdouble find_min()"
            << "\n\tYou have to supply column vectors to this function"
            << "\n\tx.nc():    " << x.nc()
        );


        T g, s;

        double f_value = f(x);
        g = der(x);

        T best_x = x;
        double best_f = f_value;

        Jacobian::projectToNullSpace(x, g);

        if (f_value == 0 || length(g) == 0)
            return f_value;

        DLIB_ASSERT(is_finite(f_value), "The objective function generated non-finite outputs");
        DLIB_ASSERT(is_finite(g), "The objective function generated non-finite outputs");

        while(stop_strategy.should_continue_search(x, f_value, g) && f_value > min_f)
        {
            s = search_strategy.get_next_direction(x, f_value, g);

            Jacobian::scale(s);
            Jacobian::projectToNullSpace(x, s);

            double alpha = line_search(
                        make_line_search_function(f,x,s, f_value),
                        f_value,
                        make_line_search_function(der,x,s, g),
                        dot(g,s), // compute initial gradient for the line search
                        search_strategy.get_wolfe_rho(), search_strategy.get_wolfe_sigma(), min_f,
                        search_strategy.get_max_line_search_iterations());

            // Take the search step indicated by the above line search
            //x += alpha*s;
            s *= alpha;
            Jacobian::scale(s);
            Jacobian::projectToNullSpace(x, s);
            x += s;

            DLIB_ASSERT(is_finite(f_value), "The objective function generated non-finite outputs");
            DLIB_ASSERT(is_finite(g), "The objective function generated non-finite outputs");

            if (f_value < best_f)
            {
                best_x = x;
                best_f = f_value;
            }
        }

        if (f_value > best_f)
        {
            x = best_x;
            f_value = best_f;
        }

        return f_value;
    }

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
//                      Functions for box constrained optimization
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    template <
        typename search_strategy_type,
        typename stop_strategy_type,
        typename funct,
        typename funct_der,
        typename T,
        typename EXP1,
        typename EXP2
        >
    double find_min_box_constrained (
        search_strategy_type search_strategy,
        stop_strategy_type stop_strategy,
        const funct& f,
        const funct_der& der,
        T& x,
        const dlib::matrix_exp<EXP1>& x_lower,
        const dlib::matrix_exp<EXP2>& x_upper
    )
    {
        /*
            The implementation of this function is more or less based on the discussion in
            the paper Projected Newton-type Methods in Machine Learning by Mark Schmidt, et al.
        */

        // make sure the requires clause is not violated
        COMPILE_TIME_ASSERT(dlib::is_matrix<T>::value);
        // The starting point (i.e. x) must be a column vector.
        COMPILE_TIME_ASSERT(T::NC <= 1);

        DLIB_CASSERT (
            dlib::is_col_vector(x) && dlib::is_col_vector(x_lower) && dlib::is_col_vector(x_upper) &&
            x.size() == x_lower.size() && x.size() == x_upper.size(),
            "\tdouble find_min_box_constrained()"
            << "\n\t The inputs to this function must be equal length column vectors."
            << "\n\t is_col_vector(x):       " << dlib::is_col_vector(x)
            << "\n\t is_col_vector(x_upper): " << dlib::is_col_vector(x_upper)
            << "\n\t is_col_vector(x_upper): " << dlib::is_col_vector(x_upper)
            << "\n\t x.size():               " << x.size()
            << "\n\t x_lower.size():         " << x_lower.size()
            << "\n\t x_upper.size():         " << x_upper.size()
        );
        DLIB_ASSERT (
            min(x_upper-x_lower) > 0,
            "\tdouble find_min_box_constrained()"
            << "\n\t You have to supply proper box constraints to this function."
            << "\n\r min(x_upper-x_lower): " << min(x_upper-x_lower)
        );


        T g, s;
        double f_value = f(x);
        g = der(x);

        T best_x = x;
        double best_f = f_value;

        Jacobian::projectToNullSpace(x, g);

        if (f_value == 0 || length(g) == 0)
            return f_value;

        DLIB_ASSERT(is_finite(f_value), "The objective function generated non-finite outputs");
        DLIB_ASSERT(is_finite(g), "The objective function generated non-finite outputs");

        // gap_eps determines how close we have to get to a bound constraint before we
        // start basically dropping it from the optimization and consider it to be an
        // active constraint.
        const double gap_eps = 1e-8;

        double last_alpha = 1;
        while(stop_strategy.should_continue_search(x, f_value, g))
        {
            s = search_strategy.get_next_direction(x, f_value, dlib::zero_bounded_variables(gap_eps, g, x, g, x_lower, x_upper));
            s = dlib::gap_step_assign_bounded_variables(gap_eps, s, x, g, x_lower, x_upper);
            Jacobian::scale(s);
            Jacobian::projectToNullSpace(x, s);

            double alpha = backtracking_line_search(
                        make_line_search_function2(clamp_function2(f,x_lower,x_upper,x,s), x, s, f_value),
                        f_value,
                        dot(g,s), // compute gradient for the line search
                        last_alpha,
                        search_strategy.get_wolfe_rho(),
                        search_strategy.get_max_line_search_iterations());

            // Do a trust region style thing for alpha.  The idea is that if we take a
            // small step then we are likely to take another small step.  So we reuse the
            // alpha from the last iteration unless the line search didn't shrink alpha at
            // all, in that case, we start with a bigger alpha next time.
            if (alpha == last_alpha)
                last_alpha = std::min(last_alpha*10,1.0);
            else
                last_alpha = alpha;

            // Take the search step indicated by the above line search
            s *= alpha;
            Jacobian::scale(s);
            Jacobian::projectToNullSpace(x, s);
            x = clamp(x + s, x_lower, x_upper);
            Jacobian::scale(s);
            Jacobian::projectToNullSpace(x, s);

            //x = x + s;
            //x = clamp(x + alpha*s, x_lower, x_upper);
            f_value = f(x);
            g = der(x);

            Jacobian::projectToNullSpace(x, g);

            DLIB_ASSERT(dlib::is_finite(f_value), "The objective function generated non-finite outputs");
            DLIB_ASSERT(dlib::is_finite(g), "The objective function generated non-finite outputs");

            if (!dlib::is_finite(f_value))
            {
                std::cout << "f value is not finite\n" << f_value << std::endl;
                break;
            }
            if (!dlib::is_finite(g))
            {
                std::cout << "g is not finite\n" << g << std::endl;
                break;
            }

            if (f_value == 0.0)
                break;

            if (f_value < best_f)
            {
                best_x = x;
                best_f = f_value;
            }
        }

        if (f_value > best_f)
        {
            x = best_x;
            f_value = best_f;
        }

        return f_value;
    }

// ----------------------------------------------------------------------------------------

}

#endif // DLIB_OPTIMIZATIOn_H_

