#import "../lib.typ": *
#set text(lang: "en")

#show: hwk.with(title: [Mid-Term Examination for Quantum Mechanism, Semester of 2025-2026(2)], author: "司徒和丞相", course: none, hwk-id: 0, stu-id: 2024001)

#problem[

  A particle of mass $m$ has the wave function:

  $ Psi(x, t) = A e^((- a [frac(m x^2, planck) + i t])) $

  where $A$ and $a$ are positive real constants.

  #sub-problem[Find $A$.]
  #sub-problem[For what potential energy function $V(x)$ is this a solution to the Schrödinger equation?]
  #sub-problem[Calculate the expectation values of $x$, $x^2$, $p$ and $p^2$.]
  #sub-problem[Find $sigma_x$ and $sigma_p$. Is their product consistent with the uncertainty principle?]

]

#solution[

  === (a) Normalize: Find $A$

  The normalization condition is $integral_(-infinity)^(infinity) abs(Psi)^2 dif x = 1$.

  The modulus squared is (the $e^(- i a t)$ and $e^(+ i a t)$ cancel):

  $ abs(Psi)^2 = A^2 exp(- 2 a m x^2 / planck) $

  Let $alpha eq.def 2 a m / planck$. Using the Gaussian integral:

  $ A^2 integral_(-infinity)^(infinity) e^(- alpha x^2) dif x = A^2 sqrt(frac(pi, alpha)) = A^2 sqrt(frac(pi planck, 2 a m)) = 1 $

  $ boxed(A = (frac(2 a m, pi planck))^(1/4)) $

  === (b) Find $V(x)$ from the Schrödinger Equation

  The time-dependent Schrödinger equation:

  $ i planck (partial Psi) / (partial t) = - frac(planck^2, 2 m) (partial^2 Psi) / (partial x^2) + V(x) Psi $

  *Left side --- time derivative:*

  $ (partial Psi) / (partial t) = - i a Psi quad arrow.r.double quad i planck (partial Psi) / (partial t) = planck a Psi $

  *Right side --- second spatial derivative:*

  $ (partial Psi) / (partial x) = - frac(2 a m x, planck) Psi $

  $ (partial^2 Psi) / (partial x^2) = (- frac(2 a m, planck) + frac(4 a^2 m^2 x^2, planck^2)) Psi $

  $ - frac(planck^2, 2 m) (partial^2 Psi) / (partial x^2) = (planck a - 2 a^2 m x^2) Psi $

  *Substituting into the Schrödinger equation:*

  $ planck a = planck a - 2 a^2 m x^2 + V(x) $

  $ boxed(V(x) = 2 a^2 m x^2) $

  This is a _harmonic oscillator_ potential $V = frac(1, 2) m omega^2 x^2$ with $omega = 2 a$, and energy eigenvalue $E = planck a = frac(1, 2) planck omega$ --- exactly the _ground state_ of the quantum harmonic oscillator.

  === (c) Expectation Values

  --- $chevron.l x chevron.r$ and $chevron.l x^2 chevron.r$

  Since $abs(Psi)^2 tilde e^(- 2 a m x^2 / planck)$ is an _even function_:

  $ boxed(chevron.l x chevron.r = 0) $

  For $chevron.l x^2 chevron.r$, use the Gaussian integral with $alpha = 2 a m / planck$:

  $ chevron.l x^2 chevron.r = A^2 dot frac(1, 2 alpha) sqrt(frac(pi, alpha)) = sqrt(frac(alpha, pi)) dot frac(1, 2 alpha) sqrt(frac(pi, alpha)) = frac(1, 2 alpha) $

  $ boxed(chevron.l x^2 chevron.r = planck / (4 a m)) $

  --- $chevron.l p chevron.r$ and $chevron.l p^2 chevron.r$

  For $chevron.l p chevron.r$ at $t = 0$ (wave function is real):

  $ chevron.l p chevron.r = - i planck integral Psi (partial Psi) / (partial x) dif x = - i planck dot (- 2 a m / planck) A^2 integral x e^(- alpha x^2) dif x = 0 $

  The integral vanishes because the integrand is _odd_:

  $ boxed(chevron.l p chevron.r = 0) $

  For $chevron.l p^2 chevron.r$, use the result from part (b):

  $ chevron.l p^2 chevron.r = - planck^2 integral Psi^* (partial^2 Psi) / (partial x^2) dif x = - planck^2 (- frac(2 a m, planck) + frac(4 a^2 m^2, planck^2) chevron.l x^2 chevron.r) $

  $ = 2 a m planck - 4 a^2 m^2 dot planck / (4 a m) = 2 a m planck - a m planck $

  $ boxed(chevron.l p^2 chevron.r = a m planck) $

  #quote[
    *Cross-check via energy:* $chevron.l E chevron.r = planck a = chevron.l p^2 chevron.r / (2 m) + chevron.l V chevron.r = frac(a m planck, 2 m) + 2 a^2 m dot planck / (4 a m) = frac(a planck, 2) + frac(a planck, 2) = a planck$
  ]

  === (d) Uncertainties and the Heisenberg Principle

  --- Standard deviations

  $ sigma_x = sqrt(chevron.l x^2 chevron.r - chevron.l x chevron.r^2) = sqrt(frac(planck, 4 a m)) = frac(1, 2) sqrt(frac(planck, a m)) $

  $ sigma_p = sqrt(chevron.l p^2 chevron.r - chevron.l p chevron.r^2) = sqrt(a m planck) $

  --- Product

  $ sigma_x dot sigma_p = frac(1, 2) sqrt(frac(planck, a m)) dot sqrt(a m planck) = frac(1, 2) sqrt(planck^2) $

  $ boxed(sigma_x sigma_p = planck / 2) $

  --- Consistency with the uncertainty principle

  The Heisenberg uncertainty principle states:

  $ sigma_x sigma_p gt.eq planck / 2 $

  Our result gives _exactly_ $planck / 2$ --- the _minimum possible value_. This is consistent with and saturates the uncertainty principle.

  #quote[
    *Why equality?* The Gaussian wave packet is the _unique_ wave function that minimizes the position-momentum uncertainty product. This wave function is precisely the ground state of the quantum harmonic oscillator --- a well-known minimum-uncertainty state.
  ]

]

---

#problem[

  For a particle in the harmonic oscillator potential, its ground state is defined via the lowering operator

  $ hat(a)_- = frac(1, sqrt(2 planck m omega)) (i hat(p) + m omega hat(x)) $

  by requiring $hat(a)_- |0 chevron.r = 0$.

  #sub-problem[Find the normalized wave function of the ground state $Psi_0(x) = chevron.l x | 0 chevron.r$ in the position space.]
  #sub-problem[For the first excited state $|1 chevron.r$, it can be obtained from $|1 chevron.r = hat(a)_+ |0 chevron.r$, where the raising operator reads $hat(a)_+ = frac(1, sqrt(2 planck m omega)) (-i hat(p) + m omega hat(x))$, find normalized $chevron.l x | 1 chevron.r$.]
  #sub-problem[Find the expectation values of the potential energy and the kinetic energy in the $n$-th stationary state.]

  #quote[
    *Hints:* $hat(a)_- |n chevron.r = sqrt(n) |n - 1 chevron.r$ and $hat(a)_+ |n chevron.r = sqrt(n + 1) |n + 1 chevron.r$.
  ]

]

#solution[

  === (a) Ground State Wave Function $Psi_0(x)$

  --- Setting up the condition

  The ground state satisfies $hat(a)_- |0 chevron.r = 0$. In position space with $hat(p) = - i planck (partial) / (partial x)$:

  $ frac(1, sqrt(2 planck m omega)) (planck (partial) / (partial x) + m omega x) Psi_0(x) = 0 $

  This gives the _first-order ODE_:

  $ (d Psi_0) / (dif x) = - frac(m omega, planck) x Psi_0 $

  --- Solving by separation

  $ (d Psi_0) / Psi_0 = - frac(m omega, planck) x dif x quad arrow.r.double quad Psi_0(x) = C exp(- frac(m omega, 2 planck) x^2) $

  --- Normalization

  $ abs(C)^2 integral_(-infinity)^(infinity) e^(- m omega x^2 / planck) dif x = 1 $

  Using $integral_(-infinity)^(infinity) e^(- alpha x^2) dif x = sqrt(pi / alpha)$ with $alpha = m omega / planck$:

  $ abs(C)^2 sqrt(frac(pi planck, m omega)) = 1 quad arrow.r.double quad C = (frac(m omega, pi planck))^(1/4) $

  $ boxed(Psi_0(x) = (frac(m omega, pi planck))^(1/4) exp(- frac(m omega x^2, 2 planck))) $

  === (b) First Excited State $Psi_1(x)$

  --- Apply the raising operator

  $ |1 chevron.r = hat(a)_+ |0 chevron.r quad arrow.r.double quad Psi_1(x) = frac(1, sqrt(2 planck m omega)) (-i hat(p) + m omega x) Psi_0(x) $

  Since $hat(p) = - i planck (partial) / (partial x)$:

  $ - i hat(p) = - planck (partial) / (partial x) $

  Computing the action on $Psi_0$:

  $ - planck (partial Psi_0) / (partial x) = - planck (- frac(m omega x, planck)) Psi_0 = m omega x Psi_0 $

  Therefore:

  $ Psi_1(x) = frac(1, sqrt(2 planck m omega)) (m omega x + m omega x) Psi_0(x) = frac(2 m omega x, sqrt(2 planck m omega)) Psi_0(x) $

  $ = sqrt(frac(2 m omega, planck)) x Psi_0(x) $

  $ boxed(Psi_1(x) = (frac(m omega, pi planck))^(1/4) sqrt(frac(2 m omega, planck)) x exp(- frac(m omega x^2, 2 planck))) $

  This is an _odd function_ times a Gaussian --- the characteristic shape of the first excited state.

  === (c) Expectation Values of KE and PE in the $n$-th State

  --- Express operators via ladder operators

  $ hat(x) = sqrt(frac(planck, 2 m omega)) (hat(a)_+ + hat(a)_-), quad hat(p) = i sqrt(frac(m omega planck, 2)) (hat(a)_+ - hat(a)_-) $

  --- Potential Energy: $chevron.l V chevron.r$

  $ hat(x)^2 = frac(planck, 2 m omega) (hat(a)_+^2 + hat(a)_+ hat(a)_- + hat(a)_- hat(a)_+ + hat(a)_-^2) $

  Taking the expectation value in state $|n chevron.r$, _only the diagonal terms survive_ ($hat(a)_+^2$ and $hat(a)_-^2$ change $n$ by $plus.minus 2$, the cross terms change it by $0$ but we need the diagonal part):

  $ chevron.l n | hat(x)^2 | n chevron.r = frac(planck, 2 m omega) (chevron.l n | hat(a)_+ hat(a)_- | n chevron.r + chevron.l n | hat(a)_- hat(a)_+ | n chevron.r) $

  Using the hints:

  $ hat(a)_+ hat(a)_- |n chevron.r = sqrt(n) hat(a)_+ |n - 1 chevron.r = n |n chevron.r $
  $ hat(a)_- hat(a)_+ |n chevron.r = sqrt(n + 1) hat(a)_- |n + 1 chevron.r = (n + 1) |n chevron.r $

  Therefore:

  $ chevron.l hat(x)^2 chevron.r_n = frac(planck, 2 m omega) (2 n + 1) $

  $ boxed(chevron.l V chevron.r_n = frac(1, 2) m omega^2 chevron.l x^2 chevron.r_n = frac(1, 4) (2 n + 1) planck omega = frac(1, 2) E_n) $

  --- Kinetic Energy: $chevron.l T chevron.r$

  $ hat(p)^2 = - frac(m omega planck, 2) (hat(a)_+^2 - hat(a)_+ hat(a)_- - hat(a)_- hat(a)_+ + hat(a)_-^2) $

  The diagonal terms give:

  $ chevron.l hat(p)^2 chevron.r_n = - frac(m omega planck, 2) (- chevron.l hat(a)_+ hat(a)_- chevron.r_n - chevron.l hat(a)_- hat(a)_+ chevron.r_n) = frac(m omega planck, 2) (2 n + 1) $

  $ boxed(chevron.l T chevron.r_n = chevron.l p^2 chevron.r_n / (2 m) = frac(1, 4) (2 n + 1) planck omega = frac(1, 2) E_n) $

  === Summary

  #set text(size: 9.5pt)
  #set table(stroke: 0.5pt, inset: 6pt)

  #align(center)[
    #table(
      columns: (auto, auto, auto),
      [*Quantity*], [*Result*], [*Fraction of $E_n$*],
      [Energy eigenvalue], $E_n = (n + frac(1, 2)) planck omega$, [---],
      [$chevron.l V chevron.r_n$], $frac(1, 4) (2 n + 1) planck omega$, [$frac(1, 2)$],
      [$chevron.l T chevron.r_n$], $frac(1, 4) (2 n + 1) planck omega$, [$frac(1, 2)$],
      [$chevron.l V chevron.r_n + chevron.l T chevron.r_n$], $(n + frac(1, 2)) planck omega = E_n$, [$1$],
    )
  ]

  #quote[
    *Physical significance:* The result $chevron.l T chevron.r_n = chevron.l V chevron.r_n = E_n / 2$ is the _quantum virial theorem_ for the harmonic oscillator --- kinetic and potential energies share the energy equally, independent of $n$.
  ]

]

---

#problem[

  Orthonormal basis $|1 chevron.r$, $|2 chevron.r$, $|3 chevron.r$ with $chevron.l i | j chevron.r = delta_(i j)$.

  $ |alpha chevron.r = i |1 chevron.r - 2 |2 chevron.r - i |3 chevron.r, quad |beta chevron.r = i |1 chevron.r + 2 |3 chevron.r $

  #sub-problem[Construct $chevron.l alpha|$ and $chevron.l beta|$.]
  #sub-problem[Find inner products $chevron.l alpha | beta chevron.r$ and $chevron.l beta | alpha chevron.r$.]
  #sub-problem[Find the matrix elements of $hat(A) = |alpha chevron.r chevron.l beta|$. Is it Hermitian?]

]

#solution[

  === (a) Construct $chevron.l alpha|$ and $chevron.l beta|$

  To get the _bra_, we take the _complex conjugate of every coefficient_ and write the dual basis in reverse order:

  $ boxed(chevron.l alpha| = - i chevron.l 1| - 2 chevron.l 2| + i chevron.l 3|) $

  $ boxed(chevron.l beta| = - i chevron.l 1| + 2 chevron.l 3|) $

  #quote[
    Note: $chevron.l beta|$ has _zero_ coefficient for $chevron.l 2|$ since $|beta chevron.r$ has no $|2 chevron.r$ component.
  ]

  === (b) Inner Products $chevron.l alpha | beta chevron.r$ and $chevron.l beta | alpha chevron.r$

  --- $chevron.l alpha | beta chevron.r$

  $ chevron.l alpha | beta chevron.r = (- i)(i) + (- 2)(0) + (i)(2) = 1 + 0 + 2 i $

  $ boxed(chevron.l alpha | beta chevron.r = 1 + 2 i) $

  --- $chevron.l beta | alpha chevron.r$

  $ chevron.l beta | alpha chevron.r = (- i)(i) + (0)(- 2) + (2)(- i) = 1 + 0 - 2 i $

  $ boxed(chevron.l beta | alpha chevron.r = 1 - 2 i) $

  --- Verification of the conjugate-symmetry property

  $ (chevron.l beta | alpha chevron.r)^* = (1 - 2 i)^* = 1 + 2 i = chevron.l alpha | beta chevron.r $

  $ boxed(chevron.l alpha | beta chevron.r = chevron.l beta | alpha chevron.r^*) $

  This confirms the fundamental property of the inner product in a complex vector space.

  === (c) Matrix Elements of $hat(A) = |alpha chevron.r chevron.l beta|$

  The matrix elements are $A_(i j) = chevron.l i | hat(A) | j chevron.r = chevron.l i | alpha chevron.r chevron.l beta | j chevron.r = alpha_i beta_j^*$.

  --- Building the coefficient vectors

  #set text(size: 9.5pt)
  #set table(stroke: 0.5pt, inset: 6pt)

  #align(center)[
    #table(
      columns: (auto, auto, auto, auto),
      [#text(size: 9pt)[ ]], [*$|1 chevron.r$*], [*$|2 chevron.r$*], [*$|3 chevron.r$*],
      [$|alpha chevron.r$: $alpha_i$], [$i$], [$- 2$], [$- i$],
      [$chevron.l beta|$: $beta_j^*$], [$- i$], [$0$], [$2$],
    )
  ]

  --- Computing all 9 elements

  $ A_(i j) = alpha_i dot beta_j^* $

  #align(center)[
    #table(
      columns: (auto, auto, auto, auto),
      [#text(size: 9pt)[ ]], [*$j = 1$ ($beta_1^* = - i$)*], [*$j = 2$ ($beta_2^* = 0$)*], [*$j = 3$ ($beta_3^* = 2$)*],
      [$i = 1$ ($alpha_1 = i$)], [$(i)(- i) = 1$], [$0$], [$(i)(2) = 2 i$],
      [$i = 2$ ($alpha_2 = - 2$)], [$(- 2)(- i) = 2 i$], [$0$], [$(- 2)(2) = - 4$],
      [$i = 3$ ($alpha_3 = - i$)], [$(- i)(- i) = - 1$], [$0$], [$(- i)(2) = - 2 i$],
    )
  ]

  --- The matrix

  $ boxed(A = mat(1, 0, 2 i; 2 i, 0, - 4; - 1, 0, - 2 i)) $

  --- Is it Hermitian?

  Check if $A^dagger = A$, i.e., $A_(j i)^* = A_(i j)$:

  #align(center)[
    #table(
      columns: (auto, auto, auto, auto),
      [*Element*], [*$A_(i j)$*], [*$A_(j i)^*$*], [*Equal?*],
      [$(1, 3)$], [$2 i$], [$(- 1)^* = - 1$], [*No*],
      [$(3, 1)$], [$- 1$], [$(2 i)^* = - 2 i$], [*No*],
    )
  ]

  Since $A_(13) = 2 i != - 1 = A_(31)^*$:

  $ boxed(hat(A) = |alpha chevron.r chevron.l beta| "is NOT Hermitian.") $

  #quote[
    *General rule:* $|alpha chevron.r chevron.l beta|$ is Hermitian _only if_ $|alpha chevron.r = e^(i phi) |beta chevron.r$ for some phase $phi$ --- which is clearly not the case here since $|alpha chevron.r$ and $|beta chevron.r$ differ in both the $|2 chevron.r$ component and the relative phases of their coefficients.
  ]

]

---

#problem[

  A three-level quantum system has Hamiltonian and observables:

  $ H = planck omega mat(1, 0, 0; 0, 1, 0; 0, 0, 2), quad A = lambda mat(0, 1, 0; 1, 0, 0; 0, 0, 2), quad B = mu mat(2, 0, 0; 0, 0, 3; 0, 3, 0) $

  #sub-problem[Find the eigenvalues and normalized eigenvectors of $H$, $A$, and $B$.]
  #sub-problem[At $t = 0$, the system is in state $|S(0) chevron.r = (c_1, c_2, c_3)^T$ with $abs(c_1)^2 + abs(c_2)^2 + abs(c_3)^2 = 1$. Find $chevron.l H chevron.r$, $chevron.l A chevron.r$, and $chevron.l B chevron.r$ at $t = 0$.]
  #sub-problem[Find $|S(t) chevron.r$ at arbitrary time $t$. If energy is measured, what are the possible outcomes and their probabilities?]

]

#solution[

  === (a) Eigenvalues and Eigenvectors

  --- Hamiltonian $H$

  $ H = planck omega mat(1, 0, 0; 0, 1, 0; 0, 0, 2) $

  Already diagonal! The eigenvalues are read directly from the diagonal:

  $ E_1 = planck omega, quad E_2 = planck omega, quad E_3 = 2 planck omega $

  Eigenvectors (the standard basis):

  $ |1 chevron.r = vec(1, 0, 0), quad |2 chevron.r = vec(0, 1, 0), quad |3 chevron.r = vec(0, 0, 1) $

  #quote[
    Note: $E_1 = E_2 = planck omega$ is a _degenerate_ eigenvalue. Any linear combination of $|1 chevron.r$ and $|2 chevron.r$ is also an eigenvector with eigenvalue $planck omega$.
  ]

  --- Observable $A$

  $ A = lambda mat(0, 1, 0; 1, 0, 0; 0, 0, 2) $

  The matrix is _block-diagonal_: a $2 times 2$ block (rows 1---2) and a $1 times 1$ block (row 3).

  *Block 1 --- the $2 times 2$ submatrix* $mat(0, 1; 1, 0)$:

  $ det mat(- nu, 1; 1, - nu) = nu^2 - 1 = 0 quad arrow.r.double quad nu = plus.minus 1 $

  *Block 2 --- the $1 times 1$ submatrix* $(2)$: eigenvalue $nu = 2$.

  #set text(size: 9.5pt)
  #set table(stroke: 0.5pt, inset: 6pt)

  #align(center)[
    #table(
      columns: (auto, auto),
      [*Eigenvalue*], [*Eigenvector*],
      [$- lambda$], [$frac(1, sqrt(2)) vec(1, - 1, 0)$],
      [$+ lambda$], [$frac(1, sqrt(2)) vec(1, 1, 0)$],
      [$2 lambda$], [$vec(0, 0, 1)$],
    )
  ]

  --- Observable $B$

  $ B = mu mat(2, 0, 0; 0, 0, 3; 0, 3, 0) $

  Again block-diagonal: a $1 times 1$ block (row 1) and a $2 times 2$ block (rows 2---3).

  *Block 1* $(2)$: eigenvalue $2 mu$.

  *Block 2* $mat(0, 3; 3, 0)$:

  $ det mat(- nu, 3; 3, - nu) = nu^2 - 9 = 0 quad arrow.r.double quad nu = plus.minus 3 $

  #align(center)[
    #table(
      columns: (auto, auto),
      [*Eigenvalue*], [*Eigenvector*],
      [$2 mu$], [$vec(1, 0, 0)$],
      [$- 3 mu$], [$frac(1, sqrt(2)) vec(0, 1, - 1)$],
      [$+ 3 mu$], [$frac(1, sqrt(2)) vec(0, 1, 1)$],
    )
  ]

  === (b) Expectation Values at $t = 0$

  Given $|S(0) chevron.r = vec(c_1, c_2, c_3)$ with $abs(c_1)^2 + abs(c_2)^2 + abs(c_3)^2 = 1$.

  --- $chevron.l H chevron.r$

  Since $H$ is diagonal:

  $ chevron.l H chevron.r = abs(c_1)^2 (planck omega) + abs(c_2)^2 (planck omega) + abs(c_3)^2 (2 planck omega) $

  $ boxed(chevron.l H chevron.r = (abs(c_1)^2 + abs(c_2)^2 + 2 abs(c_3)^2) planck omega = (1 + abs(c_3)^2) planck omega) $

  --- $chevron.l A chevron.r$

  Computing $chevron.l A chevron.r = sum_(i, j) c_i^* A_(i j) c_j$:

  $ = lambda (c_1^* c_2 + c_2^* c_1 + 2 abs(c_3)^2) $

  $ boxed(chevron.l A chevron.r = lambda (2 "Re"(c_1^* c_2) + 2 abs(c_3)^2)) $

  --- $chevron.l B chevron.r$

  $ chevron.l B chevron.r = mu (2 abs(c_1)^2 + 3 c_2^* c_3 + 3 c_3^* c_2) $

  $ boxed(chevron.l B chevron.r = mu (2 abs(c_1)^2 + 6 "Re"(c_2^* c_3))) $

  === (c) Time Evolution and Energy Measurements

  --- Finding $|S(t) chevron.r$

  Since $H$ is diagonal in the given basis, each component evolves with its own phase:

  $ e^(- i H t / planck) = mat(e^(- i omega t), 0, 0; 0, e^(- i omega t), 0; 0, 0, e^(- 2 i omega t)) $

  $ boxed(|S(t) chevron.r = vec(c_1 e^(- i omega t), c_2 e^(- i omega t), c_3 e^(- 2 i omega t))) $

  --- Energy measurement outcomes

  The possible energy eigenvalues are $E = planck omega$ and $E = 2 planck omega$.

  *For $E = planck omega$:* This eigenvalue is _degenerate_ (eigenspace spanned by $|1 chevron.r$ and $|2 chevron.r$). The total probability is:

  $ P(E = planck omega) = abs(c_1)^2 + abs(c_2)^2 $

  *For $E = 2 planck omega$:* Non-degenerate (eigenspace spanned by $|3 chevron.r$):

  $ P(E = 2 planck omega) = abs(c_3)^2 $

  --- Summary

  $ boxed(
    &E = planck omega &&"probability:" quad abs(c_1)^2 + abs(c_2)^2 \
    &E = 2 planck omega &&"probability:" quad  abs(c_3)^2
  ) $

  #quote[
    *Check:* $(abs(c_1)^2 + abs(c_2)^2) + abs(c_3)^2 = 1$
  ]

  #quote[
    *Key observation:* The degenerate eigenvalue $planck omega$ means that the system "hides" which state ($|1 chevron.r$ or $|2 chevron.r$) contributed --- we can only measure that the energy is $planck omega$, not which degenerate eigenvector was responsible.
  ]

]