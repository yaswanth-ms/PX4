/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file FigureEight.hpp
 * Helper class for lateral fixed wing position controller when flying a figure 8 loiter pattern.
 *
 */

#ifndef FIGUREEIGHT_HPP_
#define FIGUREEIGHT_HPP_

#include <cstdint>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param.h>

#include "lib/l1/ECL_L1_Pos_Controller.hpp"
#include "lib/matrix/matrix/Vector2.hpp"
#include "lib/npfg/npfg.hpp"

class FigureEight : public ModuleParams
{
public:
	/**
	 * @brief Figure eight pattern points strust
	 *
	 * Struct defining all relevant points for the figure eight pattern.
	 *
	 */
	struct FigureEightPatternPoints {
		matrix::Vector2f normalized_north_circle_offset;
		matrix::Vector2f normalized_north_entry_offset;
		matrix::Vector2f normalized_north_exit_offset;
		matrix::Vector2f normalized_south_circle_offset;
		matrix::Vector2f normalized_south_entry_offset;
		matrix::Vector2f normalized_south_exit_offset;
	};
	struct FigureEightPatternParameters {
		matrix::Vector2f center_pos_local;
		float loiter_radius;
		float loiter_minor_radius;
		float loiter_orientation;
		bool loiter_direction_counter_clockwise;
	};
	/**
	 * @brief Construct a new Figure Eight object
	 *
	 * @param[in] npfg is the reference to the parent npfg object.
	 * @param[in] l1_control is the reference to the parent l1_control object.
	 * @param[in] wind_vel is the reference to the parent wind velocity [m/s].
	 * @param[in] eas2tas is the reference to the parent indicated airspeed to true airspeed conversion.
	 */
	FigureEight(NPFG &npfg, ECL_L1_Pos_Controller &l1_control, matrix::Vector2f &wind_vel, float &eas2tas);
	/**
	 * @brief Initialize the figure eight pattern.
	 *
	 * Initialize the figure eight pattern by determining the current active segment.
	 *
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] ground_speed is the current ground speed of the vehicle in [m/s].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 */
	void initializePattern(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
			       const FigureEightPatternParameters &parameters);
	/**
	 * @brief reset the figure eight pattern.
	 *
	 * Reset the figure eight pattern such that it can be properly initialized on a new figure eight pattern.
	 *
	 */
	void resetPattern();
	/**
	 * @brief Update roll and airspeed setpoint.
	 *
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] ground_speed is the current ground speed of the vehicle in [m/s].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 * @param[in] target_airspeed is the current targeted indicated airspeed [m/s].
	 * @param[in] use_npfg flag if the npfg lateral control logic should be used.
	 */
	void updateSetpoint(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
			    const FigureEightPatternParameters &parameters, float target_airspeed, const bool use_npfg);
	/**
	 * @brief Get the roll setpoint
	 *
	 * @return the roll setpoint in [rad].
	 */
	float getRollSetpoint() const {return _roll_setpoint;};
	/**
	 * @brief Get the indicated airspeed setpoint
	 *
	 * @return the indicated airspeed setpoint in [m/s].
	 */
	float getAirspeedSetpoint() const {return _indicated_airspeed_setpoint;};


private:
	/**
	 * @brief Calculate figure eight pattern points
	 *
	 * @param[out] 	pattern_points is the output with the calculated points for the figure eight.
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 */
	void calculateFigureEightPoints(FigureEightPatternPoints &pattern_points,
					const FigureEightPatternParameters &parameters);
	/**
	 * @brief Apply lateral control logic
	 *
	 *
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] ground_speed is the current ground speed of the vehicle in [m/s].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 * @param[in] target_airspeed is the current targeted indicated airspeed [m/s].
	 * @param[in] use_npfg flag if the npfg lateral control logic should be used.
	 * @param[in] pattern_points are the relevant points defining the figure eight pattern.
	 */
	void applyControl(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
			  const FigureEightPatternParameters &parameters, float target_airspeed, const bool use_npfg,
			  const FigureEightPatternPoints &pattern_points);
	/**
	 * @brief Update active segment.
	 *
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 * @param[in] use_npfg flag if the npfg lateral control logic should be used.
	 * @param[in] pattern_points are the relevant points defining the figure eight pattern.
	 */
	void updateSegment(const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters,
			   const bool use_npfg,
			   const FigureEightPatternPoints &pattern_points);
	/**
	 * @brief calculate normalized and rotated relative vehicle position to pattern center.
	 *
	 * @param[out] pos_to_center_normalized_rotated is the calculated normalized and rotated relative vehicle position to pattern center.
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 */
	void calculatePositionToCenterNormalizedRotated(matrix::Vector2f &pos_to_center_normalized_rotated,
			const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters) const;
	/**
	 * @brief Calculate rotation angle.
	 *
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 *
	 * @return is the rotation angle of the major axis compared to north in [rad].
	 */
	float calculateRotationAngle(const FigureEightPatternParameters &parameters) const;
	/**
	 * @brief Apply circular lateral control
	 *
	 * @param[in] loiter_direction_counter_clockwise flag if the circle direction should be counter-clockwise.
	 * @param[in] normalized_circle_offset is the normalized position offset of the circle compared to the pattern center.
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] ground_speed is the current ground speed of the vehicle in [m/s].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 * @param[in] target_airspeed is the current targeted indicated airspeed [m/s].
	 * @param[in] use_npfg flag if the npfg lateral control logic should be used.
	 */
	void applyCircle(bool loiter_direction_counter_clockwise, const matrix::Vector2f &normalized_circle_offset,
			 const matrix::Vector2f &curr_pos_local,
			 const matrix::Vector2f &ground_speed, const FigureEightPatternParameters &parameters, float target_airspeed,
			 const bool use_npfg);
	/**
	 * @brief Apply path lateral control
	 *
	 * @param[in] normalized_line_start_offset is the normalized position offset of the start point of the path compared to the pattern center.
	 * @param[in] normalized_line_end_offset is the normalized position offset of the end point of the path compared to the pattern center.
	 * @param[in] curr_pos_local is the current local position of the vehicle in [m].
	 * @param[in] ground_speed is the current ground speed of the vehicle in [m/s].
	 * @param[in] parameters is the parameter set defining the figure eight shape.
	 * @param[in] target_airspeed is the current targeted indicated airspeed [m/s].
	 * @param[in] use_npfg flag if the npfg lateral control logic should be used.
	 */
	void applyLine(const matrix::Vector2f &normalized_line_start_offset, const matrix::Vector2f &normalized_line_end_offset,
		       const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
		       const FigureEightPatternParameters &parameters,
		       float target_airspeed, const bool use_npfg);

private:
	/**
	 * @brief npfg lateral control object.
	 *
	 */
	NPFG &_npfg;
	/**
	 * @brief lateral l1 control object.
	 *
	 */
	ECL_L1_Pos_Controller &_l1_control;
	/**
	 * @brief Wind velocity in [m/s].
	 *
	 */
	const matrix::Vector2f &_wind_vel;
	/**
	 * @brief Conversion factor from indicated to true airspeed.
	 *
	 */
	const float &_eas2tas;
	/**
	 * @brief Roll setpoint in [rad].
	 *
	 */
	float _roll_setpoint;
	/**
	 * @brief Indicated airspeed setpoint in [m/s].
	 *
	 */
	float _indicated_airspeed_setpoint;
	/**
	 * @brief active figure eight position setpoint.
	 *
	 */
	FigureEightPatternParameters _active_parameters;

	enum class FigureEightSegment {
		SEGMENT_UNDEFINED,
		SEGMENT_CIRCLE_NORTH,
		SEGMENT_NORTHEAST_SOUTHWEST,
		SEGMENT_CIRCLE_SOUTH,
		SEGMENT_SOUTHEAST_NORTHWEST
	};

	/**
	 * @brief Current active segment of the figure eight pattern.
	 *
	 */
	FigureEightSegment _current_segment{FigureEightSegment::SEGMENT_UNDEFINED};
	/**
	 * @brief flag if vehicle position passed circle center along major axis when on circle segment.
	 *
	 */
	bool _pos_passed_circle_center_along_major_axis;
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad
	)
};

#endif // FIGUREEIGHT_HPP_
