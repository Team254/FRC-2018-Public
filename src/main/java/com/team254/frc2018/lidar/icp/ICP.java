package com.team254.frc2018.lidar.icp;

import com.team254.frc2018.Constants;

public class ICP {

    public static final double OUTLIER_THRESH = 1.0; // multiplier of the mean distance

    public ReferenceModel reference;
    public long timeoutNs;

    public ICP(ReferenceModel ref, long timeoutMs) {
        reference = ref;
        timeoutNs = timeoutMs * 1000000;
    }

    /**
     * Applies ICP point registration to find a Transform that aligns
     * the given point cloud with the reference model. The returned
     * Transform represents the 2D pose (translation, rotation) of the
     * LIDAR sensor in the reference's coordinate system.
     * <p>
     * A result is returned after either the algorithm converges or
     * it times out.
     *
     * @param points The point cloud to align
     * @param trans  An initial guess Transform (if null, the identity is used)
     * @return The computed Transform
     */
    public Transform doICP(Iterable<Point> points, Transform trans) {
        long startTime = System.nanoTime();

        double lastMeanDist = Double.POSITIVE_INFINITY;

        trans = trans == null ? new Transform() : trans;
        while (System.nanoTime() - startTime < timeoutNs) {
            final Transform transInv = trans.inverse();

            final double threshold = lastMeanDist * OUTLIER_THRESH;
            double sumDists = 0;

            /// get pairs of corresponding points
            double SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
            double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
            int N = 0;
            for (Point p : points) {
                Point p2 = transInv.apply(p);
                Point rp = reference.getClosestPoint(p2);
                double dist = p2.getDistance(rp);
                sumDists += dist;
                if (dist > threshold) continue;
                N++;

                // Compute the terms:
                SumXa += p.x;
                SumYa += p.y;

                SumXb += rp.x;
                SumYb += rp.y;

                Sxx += p.x * rp.x;
                Sxy += p.x * rp.y;
                Syx += p.y * rp.x;
                Syy += p.y * rp.y;
            }

            lastMeanDist = sumDists / N;

            /// calculate the new transform
            // code based on http://mrpt.ual.es/reference/devel/se2__l2_8cpp_source.html#l00158
            if (N == 0) throw new RuntimeException("ICP: no matching points"); // TODO: handle this better, or avoid it
            final double N_inv = 1.0 / N;

            final double mean_x_a = SumXa * N_inv;
            final double mean_y_a = SumYa * N_inv;
            final double mean_x_b = SumXb * N_inv;
            final double mean_y_b = SumYb * N_inv;

            // Auxiliary variables Ax,Ay:
            final double Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
            final double Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;

            final double theta = (Ax == 0 && Ay == 0) ? 0.0 : Math.atan2(Ay, Ax);

            final double ccos = Math.cos(theta);
            final double csin = Math.sin(theta);

            final double tx = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
            final double ty = mean_y_a - mean_x_b * csin - mean_y_b * ccos;

            Transform prevTrans = trans;
            trans = new Transform(theta, tx, ty, csin, ccos);
            if (isConverged(prevTrans, trans)) {
                break;
            }
        }

        return trans;
    }

    private boolean isConverged(Transform prev, Transform cur) {
        return Math.abs(prev.theta - cur.theta) < Constants.kLidarICPAngleEpsilon &&
                Math.abs(prev.tx - cur.tx) < Constants.kLidarICPTranslationEpsilon &&
                Math.abs(prev.ty - cur.ty) < Constants.kLidarICPTranslationEpsilon;
    }

}