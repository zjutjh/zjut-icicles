#import "../lib.typ": *
#set text(lang: "en")

#show: hwk.with(title: [Mid-Term Examination for Thermodynamics and Statistical Mechanics 2025-2026(2)], author: "司徒和丞相", course: none, hwk-id: 0, stu-id: 2024001)

#let sinh = math.sinh
#let cosh = math.cosh

#problem[
  A system consists of $N$ *distinguishable, independent particles* obeying *Boltzmann statistics*. Each particle can occupy one of three non-degenerate energy levels: $-E$, $0$, and $E$. The system is in thermal contact with a heat reservoir at temperature $T$.

  Define $beta = 1 / (k_B T)$. The single-particle partition function is:

  $ z = e^(beta E) + 1 + e^(- beta E) = 1 + 2 cosh(beta E) $

  The occupation probabilities for the three levels are:

  $ P(-E) = e^(beta E) / z, quad P(0) = 1 / z, quad P(E) = e^(- beta E) / z $

  #sub-problem[Find the entropy at $T = 0 "K"$.]
  #sub-problem[Find the maximum entropy.]
  #sub-problem[Find the minimum energy of the system.]
  #sub-problem[Find the total partition function $Z$ for the $N$-particle system.]
  #sub-problem[Find the average energy $chevron.l U chevron.r$ of the system.]
  #sub-problem[Evaluate the integral $ integral_0^infinity (C(T)) / T dif T $.]
]

#solution[
  == (a) Entropy at $T = 0 "K"$

  As $T -> 0$, we have $beta -> infinity$, so $P(-E) -> 1$ and $P(0), P(E) -> 0$.

  Every particle occupies the ground state $-E$. There is only *one microstate* (all $N$ particles on $-E$), so:

  $ S = k_B ln 1 = boxed(0) $

  This is consistent with the *Third Law of Thermodynamics*.

  == (b) Maximum Entropy

  Maximum entropy corresponds to $T -> infinity$ ($beta -> 0$), where all three levels are *equally probable*.

  For $N$ distinguishable particles, a configuration with occupation numbers $(n_1, n_2, n_3)$ where $n_1 + n_2 + n_3 = N$ has a multiplicity:

  $ W = N! / (n_1 ! thin n_2 ! thin n_3 !) $

  $W$ is maximized when $n_1 = n_2 = n_3 = N / 3$. Using *Stirling's approximation* ($ln n! approx n ln n - n$):

  $ S_(max) &= k_B ln (N! / (N/3) !^3) \
    &approx k_B [N ln N - 3 dot N/3 dot ln(N/3)] \
    &= k_B N ln 3 $

  $ boxed(S_(max) = N k_B ln 3) $

  == (c) Minimum Energy

  The minimum energy occurs when *all* $N$ particles are in the lowest level $-E$:

  $ boxed(U_(min) = -N E) $

  == (d) Partition Function

  Since the $N$ particles are *distinguishable and independent*, the total partition function is:

  $ boxed(Z = z^N = [1 + 2 cosh(E / (k_B T))]^N) $

  == (e) Average Energy

  The average energy of a single particle:

  $ chevron.l epsilon chevron.r &= (-E) dot e^(beta E) / z + (0) dot 1 / z + (E) dot e^(- beta E) / z \
    &= E (e^(- beta E) - e^(beta E)) / z \
    &= - (2 E sinh(beta E)) / (1 + 2 cosh(beta E)) $

  For $N$ particles:

  $ boxed(chevron.l U chevron.r = - (2 N E sinh(E / (k_B T))) / (1 + 2 cosh(E / (k_B T)))) $

  == (f) Integral of $C(T)/T$

  We need to evaluate:

  $ integral_0^infinity (C(T)) / T dif T = ? $

  Since the energy levels are fixed (no volume degree of freedom), entropy depends only on temperature:

  $ dif S = (C(T)) / T dif T $

  Therefore, the integral equals the *total entropy change*:

  $ integral_0^infinity (C(T)) / T dif T = integral_0^infinity dif S = S(T = infinity) - S(T = 0) $

  Substituting results from parts (a) and (b):

  $ boxed(integral_0^infinity (C(T)) / T dif T = N k_B ln 3) $
]


#problem[
  Consider a collection of $N$ two-level systems in thermal equilibrium at temperature $T$. Each system has only two energy levels: a ground state with energy $0$ and an excited state with energy $epsilon$. Find and plot vs. temperature:

  #sub-problem[The probability that a subsystem is in the excited state]
  #sub-problem[The entropy of the entire collection]
]

#solution[

  == (a) Probability of Excited State

  For a single two-level system, the partition function is:

  $ z = 1 + e^(- beta epsilon), quad beta = (1)/(k_B T) $

By the Boltzmann distribution, the probability of occupying the excited state is:

$ boxed(P_"ex" = (e^(- beta epsilon))/(1 + e^(- beta epsilon)) = (1)/(e^( epsilon/(k_B T)) + 1)) $

=== Physical Checks

#set text(size: 9.5pt)
#set table(stroke: 0.5pt, inset: 6pt)

#align(center)[
  #table(
    columns: (auto, auto, auto),
    [*$T$*], [*$P_"ex"$*], [*Interpretation*],
    [$T arrow.r 0$], [$P_"ex" arrow.r 0$], [All systems frozen in ground state],
    [$T arrow.r infinity$], [$P_"ex" arrow.r 1/2$], [Both levels equally populated],
  )
]

=== Temperature Dependence (Sketch)

#figure(
  {
    set text(size: 9pt)
    box(stroke: 0.5pt, inset: 8pt, radius: 4pt)[
      #set align(left)
      *$P_"ex"$* vs *$T$* sketch:

      ```
      P_ex
      1/2 | · · · · · · · · · · ·  (asymptote)
          |           ·····
          |       ···
          |     ··
          |   ··
          |  ·
          | ·
          |·
          +------------------------- T
              kT ~ ε
      ```
    ]
  },
  caption: none,
)

The curve rises smoothly from $0$, has the steepest slope near $k_B T tilde epsilon$, and asymptotically approaches $1/2$.

---

== (b) Entropy of the Entire Collection

=== Derivation

*Method 1: Direct from the Gibbs (Shannon) entropy formula*

Define $p = P_"ex"$ and $q = 1 - p = P_"ground" = (1)/(1 + e^(- beta epsilon))$.

Since the $N$ systems are independent, the total entropy is:

$ boxed(S = - N k_B [p ln p + (1-p) ln(1-p)]) $

where $p = frac(1, e^( epsilon/(k_B T)) + 1)$.

#v(1em)

*Method 2: From the partition function (more formal)*

The total partition function for $N$ distinguishable subsystems:

$ Z = z^N = (1 + e^(- beta epsilon))^N $

The Helmholtz free energy:

$ F = - k_B T ln Z = - N k_B T ln(1 + e^(- epsilon/(k_B T))) $

The entropy:

$ S = - (partial F / partial T)_V $

Computing the derivative:

$ partial(F)/partial(T) = - N k_B ln(1 + e^(- beta epsilon)) - N k_B T dot (e^(- beta epsilon) dot (epsilon / k_B T^2))/(1 + e^(- beta epsilon)) $

Therefore:

$ boxed(S = N k_B ln(1 + e^(- epsilon/(k_B T))) + (N epsilon)/T dot (e^(- epsilon/(k_B T)))/(1 + e^(- epsilon/(k_B T)))) $

#quote[
  *Equivalence check:* Expanding Method 2's result with $p$ and $q$:

  $S = - N k_B [p ln p + q ln q]$ --- exactly Method 1.
]

=== Temperature Dependence (Sketch)

#figure(
  {
    set text(size: 9pt)
    box(stroke: 0.5pt, inset: 8pt, radius: 4pt)[
      #set align(left)
      *$S$* vs *$T$* sketch:

      ```
      S
      Nkln2 | · · · · · · · · · · ·  (asymptote)
            |           ·····
            |       ···
            |     ··
            |   ··
            |  ·
            | ·
            |·
            +------------------------- T
                kT ~ ε
      ```
    ]
  },
  caption: none,
)

The entropy rises from $0$ at $T = 0$ and asymptotically approaches $N k_B ln 2$ at high temperature.

#set text(size: 9.5pt)
#set table(stroke: 0.5pt, inset: 6pt)

#align(center)[
  #table(
    columns: (auto, auto, auto),
    [*$T$*], [*$S$*], [*Interpretation*],
    [$T arrow.r 0$], [$S arrow.r 0$], [Perfect order: all in ground state],
    [$T arrow.r infinity$], [$S arrow.r N k_B ln 2$], [Maximum disorder: equal occupation of 2 levels],
  )
]

#quote[
  *Note on the maximum:* The $N$ subsystems each have 2 states, giving at most $2^N$ microstates, so $S_max = k_B ln 2^N = N k_B ln 2$. This maximum is achieved at infinite temperature when both levels are equally probable.
]

---

]


#problem[

  By using Maxwell relations, prove:

  $ (partial C_p / partial p)_T = - T (partial^2 V / partial T^2)_p $

  *Given hint:* $ (partial S / partial p)_T = - (partial V / partial T)_p $
  
]

#solution[

=== Step 1 --- Definition of $C_p$

Start from the definition of the heat capacity at constant pressure:

$ C_p = T (partial S / partial T)_p $

=== Step 2 --- Differentiate both sides with respect to $p$ (at constant $T$)

$ (partial C_p / partial p)_T = (partial / partial p [T (partial S / partial T)_p])_T $

Since $T$ is held constant in the outer derivative, it passes through:

$ = T (partial / partial p (partial S / partial T)_p)_T $

=== Step 3 --- Swap the order of differentiation

Because $S$ is a _state function_, its second partial derivatives are equal (Clairaut's theorem):

$ (partial / partial p partial S / partial T)_T = (partial / partial T partial S / partial p)_p $

So:

$ (partial C_p / partial p)_T = T (partial / partial T (partial S / partial p)_T)_p $

=== Step 4 --- Apply the Maxwell relation

From the _Gibbs free energy_ $G = H - T S$, with $dif G = - S dif T + V dif p$, we get the Maxwell relation:

$ (partial S / partial p)_T = - (partial V / partial T)_p $

Substitute this in:

$ (partial C_p / partial p)_T = T (partial / partial T [ - (partial V / partial T)_p ])_p $

=== Step 5 --- Simplify
 #align(center)[
$ boxed((partial C_p / partial p)_T = - T (partial^2 V / partial T^2)_p) quad square.filled$]

---

== Summary of Logic Chain

$ C_p = T (partial S / partial T)_p &arrow.r "diff/diff p" &"swap order" &arrow.r "Maxwell" - T (partial^2 V / partial T^2)_p $

#quote[
  *Physical meaning:* This identity connects the pressure dependence of heat capacity to the _non-linearity_ of the equation of state ($V$ vs $T$). If $V$ is exactly linear in $T$ (e.g., ideal gas), the right side vanishes and $C_p$ is independent of pressure --- which is indeed the case for an ideal gas.
]

]

---

#problem[

  *Problem 4: Average Energy in the Classical Canonical Ensemble*

  A particle obeys the classical canonical distribution with energy:

  $ epsilon = frac(1, 2m)(p_x^2 + p_y^2 + p_z^2) + a x^2 + b x $

  where $a$ and $b$ are constants. Find the average energy of the particle.

]

#solution[

  === Step 1 --- Partition Function

  For a classical canonical ensemble:

  $ z = integral e^(- beta epsilon) dif^3 p dif^3 x / h^3, quad beta = 1 / (k_B T) $

  Since $epsilon$ separates into independent momentum and position parts, the integral _factorizes_:

  $ z tilde.eq z_"kinetic" dot z_"potential" $

  === Step 2 --- Momentum Part (Kinetic Energy)

  Each momentum component gives a Gaussian integral:

  $ z_(p_i) = integral_(-infinity)^(infinity) e^(- beta p_i^2 / (2m)) dif p_i = sqrt(frac(2 pi m, beta)) $

  Contribution from all three components:

  $ ln z_"kinetic" = frac(3, 2) ln frac(2 pi m, beta) $

  By the _equipartition theorem_, each quadratic momentum term contributes $frac(1, 2) k_B T$:

  $ chevron.l epsilon_k chevron.r = frac(3, 2) k_B T $

  === Step 3 --- Position Part (Potential Energy)

  $ z_x = integral_(-infinity)^(infinity) e^(- beta (a x^2 + b x)) dif x $

  _Complete the square_ in the exponent:

  $ a x^2 + b x = a (x + frac(b, 2a))^2 - frac(b^2, 4a) $

  Therefore:

  $ z_x = e^(beta b^2 / (4a)) integral_(-infinity)^(infinity) e^(- beta a (x + frac(b, 2a))^2) dif x = e^(beta b^2 / (4a)) sqrt(frac(pi, beta a)) $

  === Step 4 --- Compute the Average Energy

  $ chevron.l epsilon chevron.r = - (partial ln z) / (partial beta) $

  Combining all parts (factors of $h$, volume integrals over $y, z$ cancel):

  $ ln z = - frac(3, 2) ln beta + frac(beta b^2, 4a) - frac(1, 2) ln beta + "const." = -2 ln beta + frac(beta b^2, 4a) + "const." $

  Differentiating:

  $ chevron.l epsilon chevron.r = - frac(partial, partial beta) (-2 ln beta + frac(beta b^2, 4a)) = frac(2, beta) - frac(b^2, 4a) $

  $ boxed(chevron.l epsilon chevron.r = 2 k_B T - frac(b^2, 4a)) $

  === Verification via Equipartition Theorem

  We can decompose the result term by term:

  #set text(size: 9.5pt)
  #set table(stroke: 0.5pt, inset: 6pt)

  #align(center)[
    #table(
      columns: (auto, auto, auto),
      [*Term*], [*Contribution*], [*Method*],
      [$frac(p_x^2, 2m)$], [$frac(1, 2) k_B T$], [Equipartition (quadratic)],
      [$frac(p_y^2, 2m)$], [$frac(1, 2) k_B T$], [Equipartition (quadratic)],
      [$frac(p_z^2, 2m)$], [$frac(1, 2) k_B T$], [Equipartition (quadratic)],
      [$a x^2$], [$frac(1, 2) k_B T$], [Equipartition (quadratic)],
      [$b x$], [$- frac(b^2, 2a)$], [Linear term: shift of origin],
      [Cross-term corr.], [$+ frac(b^2, 4a)$], [From completing the square],
      [*Total*], $2 k_B T - frac(b^2, 4a)$, [ ],
    )
  ]

  === Physical Interpretation

  The potential $U(x) = a x^2 + b x$ has its minimum at $x_0 = - frac(b, 2a)$, with:

  $ U_min = - frac(b^2, 4a) $

  So the result reads:

  $ chevron.l epsilon chevron.r = underbrace(frac(3, 2) k_B T, "3D kinetic") + underbrace(frac(1, 2) k_B T, "oscillator") + underbrace(- frac(b^2, 4a), U_min) $

  The linear term $b x$ simply _shifts the equilibrium position_ to $x_0 = -b / (2a)$ without changing the oscillation frequency --- it only contributes the constant $U_min$ to the average energy.

]

---

#problem[

  *Problem 5: Fraction of Molecules Below the Most Probable Speed*

  Suppose the number of molecules in a classical ideal gas whose speed is less than the most probable speed $v_p$ is $N_p$, and the total number of gas molecules is $N$. Prove that the ratio $r eq.def N_p / N$ is _independent of the temperature_ of the gas.

]

#solution[

  === Step 1 --- Recall the Maxwell---Boltzmann Distribution

  The speed distribution of a classical ideal gas is:

  $ f(v) = 4 pi frac(m, 2 pi k_B T)^(3/2) v^2 exp(- frac(m v^2, 2 k_B T)) $

  The _most probable speed_ (found by maximizing $f(v)$) is:

  $ v_p = sqrt(frac(2 k_B T, m)) $

  === Step 2 --- Write the Ratio $r$

  $ r = N_p / N = integral_0^(v_p) f(v) dif v $

  === Step 3 --- Change of Variable: Let $u = v / v_p$

  This is the _key substitution_. Set:

  $ v = u v_p = u sqrt(frac(2 k_B T, m)), quad dif v = v_p dif u $

  When $v = 0$ $arrow.r$ $u = 0$, and $v = v_p$ $arrow.r$ $u = 1$.

  Substituting into the exponent:

  $ frac(m v^2, 2 k_B T) = frac(m, 2 k_B T) dot u^2 dot frac(2 k_B T, m) = u^2 $

  Substituting into the prefactor:

  $ 4 pi frac(m, 2 pi k_B T)^(3/2) v^2 = 4 pi frac(m, 2 pi k_B T)^(3/2) u^2 dot frac(2 k_B T, m) $

  Multiplying by $dif v = v_p dif u = sqrt(frac(2 k_B T, m)) dif u$:

  $ f(v) dif v = 4 pi frac(m, 2 pi k_B T)^(3/2) dot frac(2 k_B T, m) dot sqrt(frac(2 k_B T, m)) dot u^2 e^(- u^2) dif u $

  The $T$-dependent prefactors simplify:

  $ 4 pi dot frac(m^(3/2), (2 pi k_B T)^(3/2)) dot frac((2 k_B T)^(3/2), m^(3/2)) = frac(4 pi, (2 pi)^(3/2)) dot 2^(3/2) = frac(4, sqrt(pi)) $

  === Step 4 --- The Result

  $ boxed(r = N_p / N = frac(4, sqrt(pi)) integral_0^1 u^2 e^(- u^2) dif u) $

  _Every factor in this expression is a pure number --- there is no $T$ anywhere._ The upper limit is 1 (not a function of $T$), and the integrand $u^2 e^(- u^2)$ contains no $T$.

  Therefore, $r$ is _independent of temperature_. $square.filled$

  === Step 5 --- Numerical Value

  Evaluate the integral by parts ($w = u$, $dif w = u e^(- u^2) dif u$):

  $ integral_0^1 u^2 e^(- u^2) dif u = [- frac(u, 2) e^(- u^2)]_0^1 + frac(1, 2) integral_0^1 e^(- u^2) dif u = - frac(1, 2e) + frac(sqrt(pi), 4) "erf"(1) $

  $ approx -0.1839 + 0.4431 dot 0.8427 approx 0.1895 $

  Therefore:

  $ r = frac(4, sqrt(pi)) dot 0.1895 approx 2.257 dot 0.1895 $

  $ boxed(r approx 0.428 quad "(i.e., about 42.8%)") $

  === Physical Intuition

  Why is this ratio temperature-independent?

  #quote[
    At higher $T$, the distribution _broadens_ and shifts to larger speeds --- but $v_p$ itself shifts to a larger value by exactly the same factor. The shape of the distribution _relative to $v_p$_ is universal: the substitution $u = v / v_p$ removes all $T$-dependence. This is a self-similarity property of the Maxwell---Boltzmann distribution --- it always "looks the same" when measured in units of its own most probable speed.
  ]

]

---

#problem[

  *Problem 6: Fluctuations in Internal Energy and Heat Capacity*

  Derive that the fluctuations in internal energy at thermal equilibrium are proportional to the heat capacity, and explain why these fluctuations can be neglected under such conditions.

]

#solution[

  === Setup: Canonical Ensemble

  A system with fixed $N$ particles and volume $V$ is in thermal equilibrium with a heat bath at temperature $T$. The internal energy is not strictly constant --- it fluctuates around a mean value $chevron.l E chevron.r$. We use the canonical ensemble with $beta = 1 / (k_B T)$.

  The partition function and probability of energy state $r$ are:

  $ Z = sum_r e^(- beta E_r), quad P_r = e^(- beta E_r) / Z $

  === Part 1: Deriving the Fluctuation---Heat Capacity Relation

  --- Step 1 --- Mean Energy

  $ chevron.l E chevron.r = sum_r E_r P_r = frac(1, Z) sum_r E_r e^(- beta E_r) = - (partial ln Z) / (partial beta) $

  --- Step 2 --- Mean Square Energy

  $ chevron.l E^2 chevron.r = frac(1, Z) sum_r E_r^2 e^(- beta E_r) = frac(1, Z) (partial^2 Z) / (partial beta^2) $

  --- Step 3 --- Variance of Energy

  The variance is:

  $ sigma_E^2 = chevron.l E^2 chevron.r - chevron.l E chevron.r^2 $

  We can write this compactly using the derivative identity:

  $ sigma_E^2 = chevron.l E^2 chevron.r - chevron.l E chevron.r^2 = (partial^2 ln Z) / (partial beta^2) $

  *Proof:*

  $ (partial ln Z) / (partial beta) = frac(1, Z) (partial Z) / (partial beta) = - chevron.l E chevron.r $

  Differentiating again:

  $ (partial^2 ln Z) / (partial beta^2) = frac(partial, partial beta) (- chevron.l E chevron.r) = - (partial chevron.l E chevron.r) / (partial beta) $

  But we also need to show this equals $chevron.l E^2 chevron.r - chevron.l E chevron.r^2$. Let us compute directly:

  $ sigma_E^2 = chevron.l (E - chevron.l E chevron.r)^2 chevron.r = chevron.l E^2 chevron.r - chevron.l E chevron.r^2 $

  Using $e^(- beta E_r)$:

  $ sigma_E^2 = sum_r E_r^2 P_r - (sum_r E_r P_r)^2 = (partial^2 ln Z) / (partial beta^2) $

  This is the standard result:

  $ boxed(sigma_E^2 = chevron.l E^2 chevron.r - chevron.l E chevron.r^2 = (partial^2 ln Z) / (partial beta^2)) $

  --- Step 4 --- Connect to Heat Capacity

  The constant-volume heat capacity is:

  $ C_V = (partial chevron.l E chevron.r / partial T)_(N, V) $

  Using the chain rule $frac(partial, partial T) = frac(partial beta, partial T) frac(partial, partial beta) = - frac(1, k_B T^2) frac(partial, partial beta)$:

  $ C_V = - frac(1, k_B T^2) (partial chevron.l E chevron.r) / (partial beta) = - frac(1, k_B T^2) (- sigma_E^2) = sigma_E^2 / (k_B T^2) $

  Rearranging:

  $ boxed(sigma_E^2 = k_B T^2 C_V) $

  This is the _fluctuation---dissipation relation_ for energy: the mean-square energy fluctuation is directly proportional to the heat capacity.

  === Part 2: Why Fluctuations Are Negligible for Macroscopic Systems

  --- Relative Fluctuation

  The physically meaningful quantity is the _relative_ fluctuation:

  $ sigma_E / chevron.l E chevron.r = sqrt(k_B T^2 C_V) / chevron.l E chevron.r $

  For a macroscopic system with $N tilde 10^(23)$ particles:

  - $chevron.l E chevron.r tilde N$ (extensive)
  - $C_V tilde N$ (extensive)

  Therefore:

  $ sigma_E / chevron.l E chevron.r tilde sqrt(N) / N = 1 / sqrt(N) $

  --- Numerical Estimate

  $ sigma_E / chevron.l E chevron.r tilde 1 / sqrt(10^(23)) tilde 10^(-11.5) $

  This is an astonishingly small fraction --- the energy fluctuates by about _one part in 100 billion_.

  --- Physical Explanation

  #set text(size: 9.5pt)
  #set table(stroke: 0.5pt, inset: 6pt)

  #align(center)[
    #table(
      columns: (auto, auto),
      [*Aspect*], [*Reason*],
      [Statistical averaging], [With $N tilde 10^(23)$ particles, the central limit theorem ensures fluctuations scale as $sqrt(N)$ while the mean scales as $N$],
      [Thermal bath coupling], [The heat reservoir is macroscopic; even large energy transfers between system and bath are negligible relative to the total energy of either],
      [Measurement resolution], [No macroscopic instrument can detect a $10^(-11)$ relative fluctuation],
    )
  ]

  --- When Fluctuations Matter

  Fluctuations become significant only when:

  $ N "is small" arrow.r.long sigma_E / chevron.l E chevron.r tilde 1 / sqrt(N) tilde 1 $

  This occurs in _nanosystems_, _mesoscopic physics_, or _critical phenomena_ (where the effective number of correlated degrees of freedom diverges).

  === Summary

  $ boxed(sigma_E^2 = k_B T^2 C_V arrow.r.double sigma_E / chevron.l E chevron.r tilde 1 / sqrt(N)) $

  #quote[
    Energy fluctuations are proportional to heat capacity. For macroscopic systems ($N tilde 10^(23)$), the relative fluctuation is of order $10^(-12)$, making the thermodynamic energy effectively a _sharp, deterministic quantity_ --- which is why the canonical ensemble (fixed $T$, fluctuating $E$) gives identical predictions to the microcanonical ensemble (fixed $E$) for macroscopic systems.
  ]

]

== Summary of Results

#set text(size: 10.5pt)
#set table(stroke: 0.5pt, align: center, inset: 8pt)

#figure(
  table(
    columns: 4,
    [*Problem*], [*Part*], [*Quantity*], [*Result*],
    [1], [(a)], [Entropy at $T = 0$], [$0$],
    [1], [(b)], [Maximum entropy], [$N k_B ln 3$],
    [1], [(c)], [Minimum energy], [$-N E$],
    [1], [(d)], [Partition function], $[1 + 2 cosh(E / k_B T)]^N$,
    [1], [(e)], [Average energy], $- 2 N E sinh(E / k_B T) / [1 + 2 cosh(E / k_B T)]$,
    [1], [(f)], [Integral of $C(T)/T$], [$N k_B ln 3$],
    [2], [(a)], [Excited state prob.], $1 / (e^(epsilon / k_B T) + 1)$,
    [2], [(b)], [Entropy], $- N k_B [p ln p + (1-p) ln(1-p)]$,
    [3], [ ], [Maxwell relation proof], $(partial C_p / partial p)_T = - T (partial^2 V / partial T^2)_p$,
    [4], [ ], [Average energy], $2 k_B T - frac(b^2, 4a)$,
    [5], [ ], [Ratio $N_p / N$], [$approx 0.428$],
    [6], [ ], [Energy fluctuations], $sigma_E^2 = k_B T^2 C_V$,
  ),
  caption: none,
)
