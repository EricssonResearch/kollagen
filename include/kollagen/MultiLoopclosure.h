#include "Loopclosure.h"

#ifndef MULTILOOPCLOSURE_H
#define MULTILOOPCLOSURE_H

namespace kollagen
{

/**
 * \brief Inter-agent loop closure struct.
 *
 * Includes fields ID1 and ID2 to associate a loop-closure with the
 * corresponding agents.
 */
struct MultiLoopclosure : public Loopclosure {
  MultiLoopclosure(WalkerID ID1, WalkerID ID2, NodeID from, NodeID to,
                   double dx, double dy, double dtheta, double I11, double I22,
                   double I33)
      : Loopclosure(from, to, dx, dy, dtheta, I11, I22, I33), ID1(ID1),
        ID2(ID2){};
  /**
   * \brief ID of first agent associated with loop closure.
  */
  WalkerID ID1{};
  /**
   * \brief ID of second agent associated with loop closure.
  */
  WalkerID ID2{};
};

}  // namespace kollagen
#endif /* !MULTILOOPCLOSURE_H */
