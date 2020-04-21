/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    /**
     * Set the number of particles. Initialize all particles to
     *  - first position (based on estimates of x, y, theta and their uncertainties from GPS)
     *  - and all weights to 1.
     * Add random Gaussian noise to each particle.
     *
     */

    num_particles = 10;  // Set the number of particles

    /* Add noise to each measurement */
    std::default_random_engine gen;
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; ++i)
    {
        /*Initialize all 1000 particles to first position*/
        Particle temp_particle;
        temp_particle.id = i;
        temp_particle.x = dist_x(gen);
        temp_particle.y = dist_y(gen);
        temp_particle.theta = dist_theta(gen);
        temp_particle.weight = 1.0;

        particles.push_back(temp_particle);
        weights.push_back(temp_particle.weight);
    }

    /* Set the initialization flag to TRUE*/
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    /**
     * Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

    std::random_device rd{};
    std::mt19937 gen{ rd() };

    for (int i = 0; i < particles.size(); ++i)
    {
        Particle particle = particles[i];

        if (yaw_rate > 0.0001) /* Avoid division by zero. */
        {
            /* Update distances: https://github.com/arunchavan89/CarND-Kidnapped-Vehicle-Project/blob/master/formulae.pdf */
            particle.x = particle.x + (velocity / yaw_rate)  * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
            particle.y = particle.y + (velocity / yaw_rate)  * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));

            /* Update orientation */
            particle.theta = particle.theta + yaw_rate * delta_t;
        }
        else
        {
            /* Update distances */
            particle.x = particle.x + cos(yaw_rate) * (velocity * delta_t);
            particle.y = particle.y + sin(yaw_rate) * (velocity * delta_t);

            /* Update orientation */
            particle.theta = particle.theta;
        }

        std::normal_distribution<double>x(particle.x, std_pos[0]);
        std::normal_distribution<double>y(particle.y, std_pos[1]);
        std::normal_distribution<double>theta(particle.theta, std_pos[2]);

        particle.x = x(gen);
        particle.y = y(gen);
        particle.theta = theta(gen);

        particles[i] = particle;
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
{
    /**
     * Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */

}

double calculateLikelihood(double x, double ux, double y, double uy, double sig_x, double sig_y) {
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
    double exponent = ((x - ux) * (x - ux) / (2.0 * sig_x * sig_x)) + ((y - uy) * (y - uy) / (2.0 * sig_y * sig_y));
    double likelihood = gauss_norm * exp(-exponent);
    return likelihood;
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    const vector<LandmarkObs> &observations,
    const Map &map_landmarks)
{
    /**
     * Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */

    for (int i = 0; i < num_particles; i++) {
        Particle &p = particles[i];
        double prob = 1;
        for (int j = 0; j < observations.size(); j++) {

            double obj_x = observations[j].x;
            double obj_y = observations[j].y;

            // Transformation from particle coordinate to Map coordinate system
            double trans_x = p.x + cos(p.theta) * obj_x - sin(p.theta) * obj_y;
            double trans_y = p.y + sin(p.theta) * obj_x + cos(p.theta) * obj_y;

            vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
            double min_dist = 1000000;
            double land_x = -1;
            double land_y = -1;

            for (int k = 0; k < landmarks.size(); k++) {
                double act_x = landmarks[k].x_f;
                double act_y = landmarks[k].y_f;
                double dist = sqrt((act_x - trans_x) * (act_x - trans_x) + (act_y - trans_y) * (act_y - trans_y));

                if (dist < min_dist && dist <= sensor_range) {
                    min_dist = dist;
                    land_x = act_x;
                    land_y = act_y;
                }
            }

            // calculate likelihood using 2D - gaussian distribution
            // https://stackoverflow.com/questions/41538095/evaluate-multivariate-normal-gaussian-density-in-c
            double likelihood = calculateLikelihood(land_x, trans_x, land_y, trans_y, std_landmark[0], std_landmark[1]);
            prob *= likelihood;
        }
        p.weight = prob;
        weights[i] = prob;

    }
}

void ParticleFilter::resample()
{
    /**
     * Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles;

    for (int n = 0; n < num_particles; ++n) {
        Particle particle = particles[d(gen)];
        resampled_particles.push_back(particle);
    }
    particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
    const vector<int>& associations,
    const vector<double>& sense_x,
    const vector<double>& sense_y)
{
    // particle: the particle to which assign each listed association, 
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

double ParticleFilter::calculate_multvariate_normal_distribution(double std_landmark_x, double std_landmark_y,
    double observation_x, double observation_y, double map_x, double map_y)
{
    /* Multivariate Gaussian probability density: https://github.com/arunchavan89/CarND-Kidnapped-Vehicle-Project/blob/master/formulae.pdf */

    double result = 0.0;
    double term1 = 1 / (2 * M_PI * std_landmark_x *  std_landmark_y);
    double term2 = (observation_x - map_x) * (observation_x - map_x) / 2 * pow(std_landmark_x, 2.0);
    double term3 = (observation_y - map_y) * (observation_y - map_y) / 2 * pow(std_landmark_y, 2.0);
    double exponent_value = exp(-(term2 + term3));
    result = term1 * exponent_value;
    return result;

}