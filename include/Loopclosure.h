#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

using NodeID = int;
using WalkerID = int;

struct Loopclosure {
  Loopclosure(NodeID from, NodeID to, double dx, double dy, double dtheta,
              double I11, double I22, double I33)
      : from(from), to(to), dx(dx), dy(dy), dtheta(dtheta), I11(I11), I22(I22),
        I33(I33){};

  NodeID from;
  NodeID to;
  double dx;
  double dy;
  double dtheta;
  double I11;
  double I22;
  double I33;
};

#endif /* !LOOPCLOSURE_H */
