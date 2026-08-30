#pragma once

struct Trajectory {
    double startVelocity, endVelocity, totalTime, targetPosition;    // given
    double attackTime, decayTime, maxVelocity;                       // computed
    double k1, c2, c3, k3;    // precomputed constants

    explicit Trajectory(
        double startVelocity, double endVelocity, double attack, double decay,
        double totalTime, double targetPosition
    ) :
      startVelocity(startVelocity), endVelocity(endVelocity),
      totalTime(totalTime), targetPosition(targetPosition) {
        attackTime = totalTime * attack, decayTime = totalTime * (1 - decay),
        maxVelocity = computeMaxVelocity();
        k1          = (maxVelocity - startVelocity) / attackTime,
        c2 = computePosition1(attackTime), c3 = computePosition2(decayTime),
        k3 = (endVelocity - maxVelocity) / (totalTime - decayTime);
    }

    double getTargetVelocity(double time) const {
        if (time <= attackTime)
            return (time < attackTime) ? computeVelocity1(time) : maxVelocity;
        else if (time <= decayTime)
            return maxVelocity;
        else
            return (time < totalTime) ? computeVelocity3(time) : endVelocity;
    }

    double getTargetPosition(double time) const {
        if (time <= attackTime)
            return (time < attackTime) ? computePosition1(time) : c2;
        else if (time <= decayTime)
            return (time < decayTime) ? computePosition2(time) : c3;
        else
            return (time < totalTime) ? computePosition3(time) : targetPosition;
    }

  private:
    double inline computeMaxVelocity() const {
        return (2 * targetPosition - attackTime * startVelocity
                - (totalTime - decayTime) * endVelocity)
             / (decayTime - attackTime + totalTime);
    }

    double inline computePosition1(double time) const {
        return time * (k1 * 0.5 * time + startVelocity);
    }

    double inline computePosition2(double time) const {
        return maxVelocity * time + c2;
    }

    double inline computePosition3(double time) const {
        const double dt = time - decayTime;

        return c3 + dt * (k3 * dt + maxVelocity);
    }

    double inline computeVelocity1(double time) const {
        return k1 * time + startVelocity;
    }

    double inline computeVelocity3(double time) const {
        return k3 * (time - decayTime) + maxVelocity;
    }
};
