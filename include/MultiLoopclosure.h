#include "Loopclosure.h"

#ifndef MULTILOOPCLOSURE_H
#define MULTILOOPCLOSURE_H

struct MultiLoopclosure : public Loopclosure {
  MultiLoopclosure(WalkerID ID1, WalkerID ID2, NodeID from, NodeID to,
                   double dx, double dy, double dtheta, double I11, double I22,
                   double I33)
      : Loopclosure(from, to, dx, dy, dtheta, I11, I22, I33), ID1(ID1),
        ID2(ID2){};
  WalkerID ID1{};
  WalkerID ID2{};
};

#endif /* !MULTILOOPCLOSURE_H */
