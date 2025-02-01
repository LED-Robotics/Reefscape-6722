namespace JetsonConstants {
    const double kDistanceConfThresh = 2.0;
    const double kDistanceConfCurveExtent = 2.5;
    const double kAngularConfThresh = 11.0;
    const double kAngularConfCurveExtent = 2.5;
    // Weights for distance/angle.
    // THESE SHOULD ADD UP TO 1.0
    const double kDistanceConfWeight = 0.4;
    const double kAngularConfWeight = 0.6;

    const double kPoseConfidenceThresh = 0.7;
}
