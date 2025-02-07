namespace JetsonConstants {
    const double kDistanceConfThresh = 4.0;
    const double kDistanceConfCurveExtent = 2.0;
    const double kAngularConfThresh = 12.5;
    const double kAngularConfCurveExtent = 2.0;
    // Weights for distance/angle.
    // THESE SHOULD ADD UP TO 1.0
    const double kDistanceConfWeight = 0.5;
    const double kAngularConfWeight = 0.5;

    const double kPoseConfidenceThresh = 0.8;
    const double kRelativeConfidenceThresh = 0.7;
}
