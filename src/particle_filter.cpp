/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <stdlib.h>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	default_random_engine gen;
	
	is_initialized = true;
	num_particles = 10;
	for (int i = 0; i < num_particles; i++) {
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);	

		Particle p;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);

		p.weight = 1;
		particles.push_back(p);
		cout << "## init << " << p.x << " " << p.y << " " << p.theta << endl;
	}


}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	/*
	cout << "## delta_t: " << delta_t << endl;
	cout << "## velocity: " << velocity << endl;
	cout << "## yaw_rate: " << yaw_rate << endl;
	*/

	for (int i = 0; i < num_particles; i++) {
		Particle p = particles[i];
		//cout << "## pred_before << " << p.x << " " << p.y << " " << p.theta << endl;
	}

	default_random_engine gen;

	for (int i = 0; i < num_particles; i++) {
		// prediction		
		Particle particleUpdated = predictParticle(particles[i], delta_t, velocity, yaw_rate);
		// noise
		normal_distribution<double> dist_x(particleUpdated.x, std_pos[0]);
		normal_distribution<double> dist_y(particleUpdated.y, std_pos[1]);
		normal_distribution<double> dist_theta(particleUpdated.theta, std_pos[2]);	

		Particle particleWithAddedNoise;
		particleWithAddedNoise.x = dist_x(gen);
		particleWithAddedNoise.y = dist_y(gen);
		particleWithAddedNoise.theta = dist_theta(gen);

		particles[i] = particleWithAddedNoise;

		Particle p = particles[i];
		//cout << "## pred_after << " << p.x << " " << p.y << " " << p.theta << endl;
	}
	//for(int i; i < sizeof(std_pos); i++) {
	//	cout ";;; " << i << endl;
	//}

}

Particle ParticleFilter::predictParticle(Particle p, double dt, double v, double yaw_rate) {
	p.theta = pred_yaw(p.theta, dt, yaw_rate);
	p.x = pred_x(p.x, dt, v, p.theta, yaw_rate);
	p.y = pred_y(p.y, dt, v, p.theta, yaw_rate);
	return p;
}

double ParticleFilter::pred_yaw(double yaw, double dt, double yaw_rate) {
	return yaw + dt * yaw_rate;
}

double ParticleFilter::pred_x(double x, double dt, double v, double yaw, double yaw_rate) {
	double v_per_yr = v / yaw_rate;
	double delta = v_per_yr * ( sin(pred_yaw(yaw, dt, yaw_rate)) - sin(yaw)  );
	return x + delta;
}

double ParticleFilter::pred_y(double y, double dt, double v, double yaw, double yaw_rate) {
	double v_per_yr = v / yaw_rate;
	double delta = v_per_yr * (cos(yaw) - cos(pred_yaw(yaw, dt, yaw_rate)));
	return y + delta;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {


	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	cout << "sensor_range: " << sensor_range << endl;



	for (int i = 0; i < observations.size(); i++) {
		cout << "@ obs id \t" << observations[i].id << " x: " << observations[i].x << " y: " << observations[i].y <<endl;	


		LandmarkObs transformedLan = transformLandmark(observations[i], particles[0], map_landmarks);
		cout << "@ obs trans id \t" << transformedLan.id << " x: " << transformedLan.x << " y: " << transformedLan.y <<endl;	

	}	
	exit(0);
}

LandmarkObs ParticleFilter::transformLandmark(LandmarkObs l, Particle p, const Map &map_landmarks) {
	LandmarkObs lan;
	lan.x =  homo_x(p.x, p.theta, l.x, l.y);
	lan.y =  homo_y(p.y, p.theta, l.x, l.y);

	int nearestId = 0;
	double nearestDist = 99999999;

	for (int i = 0; i < map_landmarks.landmark_list.size(); i++) {
		double distance = sqrt(pow((lan.x - map_landmarks.landmark_list[i].x_f), 2) + pow((lan.y - map_landmarks.landmark_list[i].y_f), 2));
		if (distance < nearestDist) {
			nearestId = map_landmarks.landmark_list[i].id_i;
			nearestDist = distance;
		}
		//cout << "!map: " << map_landmarks.landmark_list[i].id_i << " x : " << map_landmarks.landmark_list[i].x_f << " y : " << map_landmarks.landmark_list[i].y_f << " D: " << distance << endl;
	}
	lan.id = nearestId;

	return lan;
}


double ParticleFilter::homo_x(double xp, double theta_p, double xc, double yc) {
	return xp + (cos(theta_p) * xc) - (sin(theta_p) * yc);
}

double ParticleFilter::homo_y(double yp, double theta_p, double xc, double yc) {
	return yp + (sin(theta_p) * xc) + (sin(theta_p) * yc);
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
