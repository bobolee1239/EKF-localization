# Localization with Extend Kalman Filter

__DESCRIPTION__: Homework 5 of Mobile Robotic <br/>
__AUTHOR__ &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: Tsung-Han Brian Lee <br />
__LICENSE__&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; : MIT

---

### Simulate under unkown correspondency case

```matlab
16 KNOWN_CORRESPONDENCY   = false;
```

### Change Watch Scope

```matlab
17 WATCH_SCOPE            = <value_you_want>;
```

### Plot Covariance with different confident
In `plotGaussian.m`, modify <confidence> to meet your need.
```matlab
33 chisquare_val = sqrt(chi2inv(<confidence>, 2));
```

---

## REFERENCE

* Dynamic Systems amd Control Lab @ National Tsing Hua University
