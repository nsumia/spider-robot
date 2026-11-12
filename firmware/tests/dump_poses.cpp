// Dumps the joint poses the firmware gaits actually command (JSON) so the CAD interference/ground checks can consume them.
#include <stdio.h>
#include <math.h>
#include "gait.h"
using namespace spider;

int main(int argc, char** argv) {
  FILE* f = argc > 1 ? fopen(argv[1], "w") : stdout;
  if (!f) return 1;
  fprintf(f, "{\"leg_order\": [\"FL\",\"FR\",\"RR\",\"RL\"], \"poses\": [\n");
  bool first = true;
  auto emit = [&](const char* name, const JointAngles* j) {
    fprintf(f, "%s {\"name\": \"%s\", \"psi\": [%.4f, %.4f, %.4f, %.4f], \"q\": [%.4f, %.4f, %.4f, %.4f]}", first ? "" : ",\n", name,
            j[0].psi, j[1].psi, j[2].psi, j[3].psi, j[0].q, j[1].q, j[2].q, j[3].q);
    first = false;
  };
  JointAngles st[4], sit[4];
  for (int i = 0; i < 4; i++) { st[i] = JointAngles{0, 0}; sit[i] = JointAngles{0, 55}; }
  emit("stand", st); emit("sit", sit);
  const char* gn[2] = {"creep", "trot"};
  const char* cn[4] = {"fwd", "back", "left", "right"};
  const float cf[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (int g = 0; g < 2; g++)
    for (int c = 0; c < 4; c++)
      for (int k = 0; k < 16; k++) {
        JointAngles j[4]; bool sw[4];
        GaitType t = g ? GAIT_TROT : GAIT_CREEP;
        gaitSample(t, defaultParams(t), k / 16.0f, cf[c][0], cf[c][1], j, sw);
        char nm[48]; snprintf(nm, sizeof nm, "%s_%s_%02d", gn[g], cn[c], k);
        emit(nm, j);
      }
  fprintf(f, "\n]}\n");
  if (f != stdout) fclose(f);
  return 0;
}
