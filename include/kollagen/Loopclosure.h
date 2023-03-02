#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

using NodeID = int;
using WalkerID = int;

namespace kollagen
{

/**
 * \brief Loop closure struct.
 */
struct Loopclosure {
  Loopclosure(NodeID from, NodeID to, double dx, double dy, double dtheta,
              double I11, double I22, double I33)
      : from(from), to(to), dx(dx), dy(dy), dtheta(dtheta), I11(I11), I22(I22),
        I33(I33){};

  /**
   * \brief ID of node from which the loop closure originates.
  */
  NodeID from;
  /**
   * \brief ID of node to where the loop closure ends.
  */
  NodeID to;
  /**
   * \brief "Measured" difference in x between the nodes.
  */
  double dx;
  /**
   * \brief "Measured" difference in y between the nodes.
  */
  double dy;
  /**
   * \brief "Measured" difference in angle between the nodes.
  */
  double dtheta;
  /**
   * \brief Information matrix entry I11 [1/(length unit)^2].
  */
  double I11;
  /**
   * \brief Information matrix entry I22 [1/(length unit)^2].
  */
  double I22;
  /**
   * \brief Information matrix entry I33 [1/radians^2].
  */
  double I33;
};

}  // namespace kollagen
#endif /* !LOOPCLOSURE_H */
