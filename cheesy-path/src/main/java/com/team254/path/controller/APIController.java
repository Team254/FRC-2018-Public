package com.team254.path.controller;

import java.net.URLDecoder;
import java.util.ArrayList;

import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;
import org.springframework.web.bind.annotation.RestController;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.spline.QuinticHermiteSpline;
import com.team254.lib.spline.Spline;
import com.team254.lib.spline.SplineGenerator;

@RestController
@RequestMapping("api")
public class APIController {
	@RequestMapping(value = "/calculate_splines", method = RequestMethod.POST)
	public @ResponseBody String calcSplines(@RequestBody String message) {
		message = message.substring(0, message.length() - 1);

        try {
            message = URLDecoder.decode(message, "UTF-8");
        } catch (Exception e) {
            e.printStackTrace();
        }

        ArrayList<Pose2d> points = new ArrayList<>();
        for (String pointString : message.split(";")) {
            String[] pointData = pointString.split(",");

            int x = pointData[0].equals("NaN") ? 0 : Integer.parseInt(pointData[0]);
            int y = pointData[1].equals("NaN") ? 0 : Integer.parseInt(pointData[1]);
            int heading = pointData[2].equals("NaN") ? 0 : Integer.parseInt(pointData[2]);

            points.add(new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(heading)));
        }

        ArrayList<QuinticHermiteSpline> mQuinticHermiteSplines = new ArrayList<>();
        ArrayList<Spline> mSplines = new ArrayList<>();
        ArrayList<Pose2dWithCurvature> positions = new ArrayList<>();
        if (points.size() < 2) {
            return "no";
        } else {
            for (int i = 0; i < points.size() - 1; i++) {
                mQuinticHermiteSplines.add(new QuinticHermiteSpline(points.get(i), points.get(i + 1)));
            }

            QuinticHermiteSpline.optimizeSpline(mQuinticHermiteSplines);

            for (QuinticHermiteSpline mQuinticHermiteSpline : mQuinticHermiteSplines) {
                mSplines.add(mQuinticHermiteSpline);
            }

            positions.addAll(SplineGenerator.parameterizeSplines(mSplines));
        }

        String json = "{\"points\":[";
        for (Pose2dWithCurvature pose : positions) {
            json += poseToJSON(pose) + ",";
        }

        return json.substring(0, json.length() - 1) + "]}";
    }

    private String poseToJSON(Pose2dWithCurvature pose) {
	    double x = pose.getTranslation().x();
	    double y = pose.getTranslation().y();
	    double rotation = pose.getRotation().getRadians();
	    double curvature = pose.getCurvature();

	    return "{\"x\":" + x + ", \"y\":" + y + ", \"rotation\":" + rotation + ", \"curvature\":" + curvature + "}";
    }
}