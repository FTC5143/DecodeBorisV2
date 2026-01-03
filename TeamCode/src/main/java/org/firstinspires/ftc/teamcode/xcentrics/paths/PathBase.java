package org.firstinspires.ftc.teamcode.xcentrics.paths;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public abstract class PathBase {

    public abstract void buildPaths();

    public abstract PathChain scorePreload();

    public abstract PathChain getFirstPattern();

    public abstract PathChain emptyGate();

    public abstract PathChain scoreFirstPattern();

    public abstract PathChain goToSecondPattern();

    public abstract PathChain getSecondPattern();

    public abstract PathChain scoreSecondPreload();

    public abstract PathChain goToThirdPattern();

    public abstract PathChain getThirdPattern();

    public abstract PathChain scoreThirdPattern();

    public abstract PathChain park();
}
