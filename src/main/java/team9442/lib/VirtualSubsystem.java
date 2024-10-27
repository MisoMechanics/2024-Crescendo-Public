// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package team9442.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a subsystem unit that requires a periodic callback but not a hardware mutex. */
public abstract class VirtualSubsystem extends SubsystemBase {

    /** This method is called periodically once per loop cycle. */
    public abstract void periodic();
}
