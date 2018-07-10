package com.team254.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();
}
