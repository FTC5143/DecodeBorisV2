package org.firstinspires.ftc.teamcode.xcentrics.paths;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public abstract class PathBase {

    public abstract void buildPaths(int pattern);

    public abstract PathChain scorePreload();

    public abstract PathChain getFirstPattern();

    public abstract PathChain scoreFirstPattern();

    public abstract PathChain getSecondPattern();

    public abstract PathChain park();
}
