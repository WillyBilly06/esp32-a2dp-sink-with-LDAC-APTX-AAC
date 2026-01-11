#include "ldac.h"
__inline static void dequant_spectrum_core_ldac(AC* p_ac, int iqu) {
  int i;
  int isp = ga_isp_ldac[iqu];
  int nsps = ga_nsps_ldac[iqu];
  int* p_qspec = p_ac->a_qspec + isp;
  const SCALAR iqf = ga_iqf_ldac[p_ac->a_idwl1[iqu]];
  const SCALAR sf = ga_sf_ldac[p_ac->a_idsf[iqu]];
  SCALAR* p_nspec = p_ac->p_acsub->a_spec + isp;

  for (i = 0; i < nsps; i++) {
    p_nspec[i] = (SCALAR)p_qspec[i] * iqf * sf;
  }
  return;
}
DECLFUNC void dequant_spectrum_ldac(AC* p_ac) {
  int iqu;
  int nqus = p_ac->p_ab->nqus;
  for (iqu = 0; iqu < nqus; iqu++) {
    dequant_spectrum_core_ldac(p_ac, iqu);
  }
}
__inline static void dequant_residual_core_ldac(AC* p_ac, int iqu) {
  int i;
  int nsps = ga_nsps_ldac[iqu];
  int isp = ga_isp_ldac[iqu];
  int* p_rspec = p_ac->a_rspec + isp;
  const SCALAR iqf = ga_iqf_ldac[p_ac->a_idwl2[0]];
  const SCALAR sf = ga_sf_ldac[p_ac->a_idsf[0]];
  const SCALAR rsf = ga_rsf_ldac[LDAC_MAXIDWL1];
  const SCALAR tmp = rsf * iqf * sf;
  SCALAR* p_nspec = p_ac->p_acsub->a_spec + isp;

  for (i = 0; i < nsps; i++) {
    p_nspec[i] += (tmp * (SCALAR)p_rspec[i]);
  }
  return;
}
DECLFUNC void dequant_residual_ldac(AC* p_ac) {
  int nqus;
  int iqu;
  nqus = p_ac->p_ab->nqus;
  for (iqu = 0; iqu < nqus; iqu++) {
    if (p_ac->a_idwl2[iqu] > 0) {
      dequant_residual_core_ldac(p_ac, iqu);
    }
  }
}

DECLFUNC void clear_spectrum_ldac(AC* p_ac, int nsps) {
  clear_data_ldac(p_ac->p_acsub->a_spec, sizeof(SCALAR) * nsps);
}
