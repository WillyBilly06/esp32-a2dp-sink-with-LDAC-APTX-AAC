#include "ldac.h"
/***************************************************************************************************
    Calculate Additional Word Length Data
***************************************************************************************************/
void calc_add_word_length_ldac(AC* p_ac) {
  int iqu;
  int nqus = p_ac->p_ab->nqus;
  int dif;
  int* p_idsf = p_ac->a_idsf;
  int* p_addwl = p_ac->a_addwl;

  clear_data_ldac(p_addwl, LDAC_MAXNQUS * sizeof(int));

  if (p_ac->p_ab->grad_mode != LDAC_MODE_0) {
    for (iqu = 1; iqu < nqus; iqu++) {
      dif = p_idsf[iqu] - p_idsf[iqu - 1];

      if (dif > 0) {
        if (dif > 5) {
          p_addwl[iqu] += 5;
        } else if (dif > 4) {
          p_addwl[iqu] += 4;
        } else if (dif > 3) {
          p_addwl[iqu] += 3;
        } else if (dif > 2) {
          p_addwl[iqu] += 2;
        } else if (dif > 1) {
          p_addwl[iqu] += 1;
        }
      } else {
        if (dif < -5) {
          p_addwl[iqu - 1] += 5;
        } else if (dif < -4) {
          p_addwl[iqu - 1] += 4;
        } else if (dif < -3) {
          p_addwl[iqu - 1] += 3;
        } else if (dif < -2) {
          p_addwl[iqu - 1] += 2;
        } else if (dif < -1) {
          p_addwl[iqu - 1] += 1;
        }
      }
    }
  }

  return;
}
DECLFUNC void reconst_gradient_ldac(AB* p_ab, int lqu, int hqu) {
  int grad_qu_h;
  int grad_qu_l;
  int grad_os_l;
  int grad_os_h;
  int tmp;
  int* p_grad;
  const unsigned char* p_t;

  grad_qu_h = p_ab->grad_qu_h;
  grad_qu_l = p_ab->grad_qu_l;
  grad_os_l = p_ab->grad_os_l;
  grad_os_h = p_ab->grad_os_h;
  tmp = grad_qu_h - grad_qu_l;
  p_grad = p_ab->a_grad;
  int iqu = lqu;
  for (iqu = 0; iqu < grad_qu_h; iqu++) {
    p_grad[iqu] = -grad_os_l;
  }
  for (iqu = grad_qu_h; iqu < hqu; iqu++) {
    p_grad[iqu] = -grad_os_h;
  }
  if (tmp > 0) {
    p_t = gaa_resamp_grad_ldac[tmp - 1];

    tmp = grad_os_h - grad_os_l;
    if (tmp > 0) {
      tmp = tmp - 1;
      for (iqu = grad_qu_l; iqu < grad_qu_h; iqu++) {
        p_grad[iqu] -= ((*p_t++ * tmp) >> 8) + 1;
      }
    } else if (tmp < 0) {
      tmp = -tmp - 1;
      for (iqu = grad_qu_l; iqu < grad_qu_h; iqu++) {
        p_grad[iqu] += ((*p_t++ * tmp) >> 8) + 1;
      }
    }
  }
}
DECLFUNC void reconst_word_length_ldac(AC* p_ac) {
  int iqu;
  int idwl1, idwl2;
  int *p_idsf, *p_addwl, *p_idwl1, *p_idwl2, *p_grad;
  AB* p_ab = p_ac->p_ab;
  int hqu = p_ab->nqus;
  int grad_mode = p_ab->grad_mode;
  p_grad = p_ab->a_grad;
  p_idsf = p_ac->a_idsf;
  p_addwl = p_ac->a_addwl;
  p_idwl1 = p_ac->a_idwl1;
  p_idwl2 = p_ac->a_idwl2;

  if (grad_mode == LDAC_MODE_0) {
    for (iqu = 0; iqu < hqu; iqu++) {
      idwl1 = p_idsf[iqu] + p_grad[iqu];
      p_idwl1[iqu] = idwl1;
      if (p_idwl1[iqu] <= 0) {
        p_idwl1[iqu] = 1;
      }
    }
  } else if (grad_mode == LDAC_MODE_1) {
    for (iqu = 0; iqu < hqu; iqu++) {
      idwl1 = p_idsf[iqu] + p_grad[iqu] + p_addwl[iqu];
      if (idwl1 > 0) {
        idwl1 = idwl1 >> 1;
      }
      p_idwl1[iqu] = idwl1;
      if (p_idwl1[iqu] <= 0) {
        p_idwl1[iqu] = 1;
      }
    }
  } else if (grad_mode == LDAC_MODE_2) {
    for (iqu = 0; iqu < hqu; iqu++) {
      idwl1 = p_idsf[iqu] + p_grad[iqu] + p_addwl[iqu];
      if (idwl1 > 0) {
        idwl1 = (idwl1 * 3) >> 3;
      }
      p_idwl1[iqu] = idwl1;
      if (p_idwl1[iqu] <= 0) {
        p_idwl1[iqu] = 1;
      }
    }
  } else if (grad_mode == LDAC_MODE_3) {
    for (iqu = 0; iqu < hqu; iqu++) {
      idwl1 = p_idsf[iqu] + p_grad[iqu] + p_addwl[iqu];
      if (idwl1 > 0) {
        idwl1 = idwl1 >> 2;
      }
      p_idwl1[iqu] = idwl1;
      if (p_idwl1[iqu] <= 0) {
        p_idwl1[iqu] = 1;
      }
    }
  }
  if (p_ab->nadjqus > 0) {
    for (iqu = 0; iqu < p_ab->nadjqus; iqu++) {
      p_idwl1[iqu] += 1;
    }
  }

  for (iqu = 0; iqu < hqu; iqu++) {
    p_idwl2[iqu] = 0;
    if (p_idwl1[iqu] > LDAC_MAXIDWL1) {
      p_idwl2[iqu] = p_idwl1[iqu] - LDAC_MAXIDWL1;
      if (p_idwl2[iqu] >= LDAC_MAXIDWL2) {
        p_idwl2[iqu] = LDAC_MAXIDWL2;
      }
      p_idwl1[iqu] = LDAC_MAXIDWL1;
    }
  }
}
