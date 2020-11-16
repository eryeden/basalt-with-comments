
# How to understand `d_rel_d_h`, `d_rel_d_t` in `BundleAdjustmentBase::computeRelPose`.

Thank you for sharing a grate software!!
Basaltのコードを最近調べているのですが、`BundleAdjustmentBase::computeRelPose`内の`d_rel_d_h`, `d_rel_d_t`の計算の理解に手間取っています。自分の理解のどこが間違っているか教えていただけるとありがたいです。
I read code of basalt recently, but I can't understand the calculation of `d_rel_d_h` and `d_rel_d_t` at `BundleAdjustmentBase::computeRelPose`.
Could you please point out the mistakes of my derivation?

`d_rel_d_h`:

`d_rel_d_h`は、`T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h`の`T_w_i_h`についてのLeft jacobianと考えています。
よって、


I believe that `d_rel_d_h` is Left jacobian for `T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h` on `T_w_i_h`. Therefore, 
$$
\begin{aligned}
\mathrm{d\_rel\_d\_h} &=
\frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh}T_{ICh}}{D~ T_{WIh} T_{ICh}} 
\frac{{}^\mathcal{E} D~ T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&=\boldsymbol{A}d_{(T_{ICt}^{-1} T_{WIt}^{-1})} \boldsymbol{I}_{66}
\end{aligned}

$$

But, the calculation of `d_rel_d_h` on the code is completely different from my derivation...

しかしながらコード上の`d_rel_d_h`の式は完全に違っています。

`d_rel_d_t`:

I also believe that `d_rel_d_t` is Left jacobian for `T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h` on `T_w_i_t`. Therefore, 


`d_rel_d_t`は`T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h`の`T_w_i_t`についてのLeft jacobianと考えています。
よって、
$$
\begin{aligned}
\mathrm{d\_rel\_d\_t} &= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIt}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh}  }{D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}{D~ T_{WIt}^{-1}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1}}{D~ T_{WIt}}\\
&= \boldsymbol{A}d_{T_{ICt}^{-1}} \boldsymbol{I}_{66} (-\boldsymbol{A}d_{T_{WIt}^{-1}})    
\end{aligned}

$$

This one is a bit close, but it's different from the calculation in the code...

こちらも少し惜しいのですが、コード上の計算と違っています。
間違っているところ、考えを指摘していただけるとありがたいです。


Note:.


$$
\mathrm{Ad}_{M} = \begin{bmatrix}
    R & [\boldsymbol{t}]_\times R\\
    \boldsymbol{0}_{33} & R
\end{bmatrix}
$$

where

$$
M = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix} \in SE(3)
$$

Jacobian $\frac{{{}^{\mathcal{E}}}D f(X)}{D X}$ is taken from equation (44) in http://arxiv.org/abs/1812.01537.



### Post

Thank you for sharing a great software!!

I read code recently, but I can't understand the calculation of `d_rel_d_h` and `d_rel_d_t` in `BundleAdjustmentBase::computeRelPose`.
Could you please point out the mistakes of my derivation?

`d_rel_d_h`:

I believe that `d_rel_d_h` is Left jacobian for `T_i_c_t^-1 T_w_i_t^-1 T_w_i_h T_i_c_h` on `T_w_i_h`. 
Therefore, 
```math
\begin{aligned}
\mathrm{d\_rel\_d\_h} &=
\frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh}T_{ICh}}{D~ T_{WIh} T_{ICh}} 
\frac{{}^\mathcal{E} D~ T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&=\boldsymbol{A}d_{(T_{ICt}^{-1} T_{WIt}^{-1})} \boldsymbol{I}_{66}
\end{aligned}
```
But, the calculation of `d_rel_d_h` on the code is completely different from my derivation...

`d_rel_d_t`:

I also believe that `d_rel_d_t` is Left jacobian for `T_i_c_t^-1 T_w_i_t^-1 T_w_i_h T_i_c_h` on `T_w_i_t`. 
Therefore, 

```math
\begin{aligned}
\mathrm{d\_rel\_d\_t} &= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIt}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh}  }{D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}{D~ T_{WIt}^{-1}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1}}{D~ T_{WIt}}\\
&= \boldsymbol{A}d_{T_{ICt}^{-1}} \boldsymbol{I}_{66} (-\boldsymbol{A}d_{T_{WIt}^{-1}})    
\end{aligned}
```
This one is a bit close, but it's different from the calculation in the code...

Note:

- Adjoint $`M \in SE(3)`$ is
```math
\mathrm{Ad}_{M} = \begin{bmatrix}
    R & [\boldsymbol{t}]_\times R\\
    \boldsymbol{0}_{33} & R
\end{bmatrix}
```

where

```math
M = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}.
```

- Jacobian $`\frac{{{}^{\mathcal{E}}}D f(X)}{D X}`$ is taken from equation (44) in http://arxiv.org/abs/1812.01537.



# Reply
## Nikolaus
- 導出は見た感じOK
- しかしBasaltでは decoupled left incrementを使っているので、Chain ruleをもう一回かける必要がある。
- Decouled left incrementについてのでFull SE3のincrementの微分はこれ
- tは、微分しようとしているSE3のTranslation
- おれの結果にあとからこれを掛けると、Code上と同じ数式になる。
- 数値微分でも、この結果は検証可能。TestコードをFull SE3のIncrementに変更しないと、Testに失敗すると思うよ。


## Kazuki
- 自分の導出に、Decoupledへの変換行列をかけたら、Codeの数式を得られた。
- また、TestをFull SE3 left incrementに変更して、Full SE3のDerivationを数値微分で検証すると、Passすることもわかった。
- 一方、 Decoupled left incrementや、変換行列の導出についてはよく理解できていない。以下の解釈でOK？
- 俺の解釈：
Decoupled left incrementは変換行列を掛けることで、Full SE3 left incrementと同じ結果をえることができている。
数式で書くと以下のようなイメージ。

関数$f(X) \in SE3, X \in SE3$であるとする。
また、
$\oplus$ : SE3 full left increment, 
$\boxplus$ : Decoupled left increment
とする。

このとき、$\boldsymbol{\tau} = [\boldsymbol{v}, \boldsymbol{\omega}]^T \rightarrow \boldsymbol{0}$であるとき、
$$
\begin{aligned}
f(\tau \oplus X) \rightarrow &\frac{{}^{\mathcal{E}}D f(X)}{D X} \boldsymbol{\tau} \oplus X \\
&= \frac{{}^{\mathcal{E}}D f(X)}{D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}
 \boldsymbol{\tau} \boxplus X
\end{aligned}

$$
として、Full SE3 incrementの代わりにDecoupled left incrementを使ってSE3のIncrementを計算していると理解した。



- Full SE3 left incrementに対して、計算量的には有利なのは理解できる。
- もし、Decoupled incrementや、他の<R^3, SO3>, [R^3 x SO3]について参考になる文献があれば教えてほしい。


## Post

I'm sorry for late reply.
Thank you so much for your kindly answer!!

I could successfully get the expression in the code by post-multiplying the matrix to my derivations.
And also, I conformed that the jacobian test with my derivations passes when replacing decoupled left increment to full SE3 left increment!

However, decoupled left increment and the conversion matrix 
$\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}$
are still unclear to me.
Decoupled left increment with conversion matrix dose the same calculation as Full SE3 left increment? like this,

$$
\begin{aligned}
f(\tau \oplus X) \rightarrow &\frac{{}^{\mathcal{E}}D f(X)}{D X} \boldsymbol{\tau} \oplus X \\
&= \frac{{}^{\mathcal{E}}D f(X)}{D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}
 \boldsymbol{\tau} \boxplus X,
\end{aligned}
$$
where,
$$
\boldsymbol{\tau} = \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix} \rightarrow \boldsymbol{0},
$$
$$
f: SE3 \rightarrow SE3,\\
X = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix},
$$

$\oplus$ : Full SE3 left increment,

$\boxplus$ : Decoupled left increment,

$\frac{{}^{\mathcal{E}}D f(X)}{D X}$ : Full SE3 Left jacobian of f(X) w.r.t. X.




I will appreciate you if you could recommend me some references about decoupled left increment.

# Reply 2

## Nikolaus Demmel

Nikolausからの返答：

-  "Full SE3 Left jacobian of f(X) w.r.t. X."?の記号はどことを参考にしている？

なでのここでChain ruleを適用するのかの厳密な導出ではありませんが、Conversion matrixは以下のように導けます。
これは、関数
$$ f: SE(3) \rightarrow SE(3)$$
の微分をどうやって定義するかによって、決まっていて、Basaltでは、以下の定義を利用している。
$$
\frac{\partial f(X)}{\partial X} := \lim_{\tau \rightarrow 0} \frac{\mathrm{log_d}(f(X \oplus \tau) f(X)^{-1})}{\tau},
$$
ここで、$\mathrm{log_d}$は、Decoupled SE3 Logであって、以下の定義になる。
$$
\mathrm{lod_d}(X) := \begin{bmatrix}
    t\\
    \log(R)
\end{bmatrix}
$$
ここでFull logを使っても極限を計算するときに、二次の項は消えるはずなので同じ結論になるはず。

この微分の定義で恒等写像$f(X)=X$のJacbobianを計算すると、
$$
\frac{\partial f(X)}{\partial X} := \begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}
$$
になる。



PS:

Decoupled left incrementをどこかで見たことはない。また、ここで使っているような計算を見たこともない。
また、Decoupled left incrementは、必ずしも広く使われている名前なわけではないと思う。
また、$T \oplus \tau:=\begin{bmatrix}
    \exp{\omega} & \nu\\
    0 & 1
\end{bmatrix}$
をDecoupled left incrementと呼ぶ人もいる。これあ実はFull left incrementと同じJacobianになる。

## KK

返答内容：

返信が遅くなってしまいとても申し訳ないです。
とても役に立つご返信ありがとうございます！！！
自分のJacobianの記号は、この論文を参考にしています。

$
\frac{^{\mathcal{E}}D f(X)}{D X}
$
ですが、以下の定義になっています。
$$
\frac{^{\mathcal{E}}D f(X)}{D X} = 
\lim_{\boldsymbol{^{\mathcal{E}}\tau}\rightarrow 0} 
\frac{f(^{\mathcal{E}}\tau \oplus X)\ominus f(X)}
{^{\mathcal{E}}\tau},
$$
where

- $\mathcal{E} = \begin{bmatrix}
    I_3 & \boldsymbol{0}_{31}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)$ : Origin of SE3.
- $X=\begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)$
- $^{\mathcal{E}}\boldsymbol{\tau} = \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix} \in T_\mathcal{E} SE(3)$ : The vector representation of $\boldsymbol{\tau}\hat{} \in \mathfrak{se}3$
- $^{\mathcal{E}}\tau \oplus X = \begin{bmatrix}
    \mathrm{Exp}(\boldsymbol{\omega}) & \boldsymbol{V}(\boldsymbol{\omega})\boldsymbol{v} \\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}
\begin{bmatrix}
R & \boldsymbol{t}\\
\boldsymbol{0}_{13} & 1
\end{bmatrix}
$ : Full left increment of SE3 ([1] eq. 27, eq. 172)
- $A \ominus B = \mathrm{Log}(AB^{-1})$, $A, B \in SE(3)$ : from [1] eq.28
- $\mathrm{Log}(X) = \begin{bmatrix}
    \boldsymbol{V}(\boldsymbol{w})^{-1} \boldsymbol{t}\\
    Log_{SO3}(R)
\end{bmatrix}$ : from [1] eq.173


Jacobianの導出や、Decoupled left incrementについての情報ありがとうございました。
頂いた導出のおかげて、このIssueで疑問だった$\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times\\
    0_3 & I_3
\end{bmatrix}$
が登場する理由を理解できたように思います。
Full SE3 left JacobianをBasaltで使っているDecoupled log, Decoupled left incrementのJacobianへの変換は、以下のようなCahin ruleがあると自分は理解しました。

$$
\begin{aligned}
\frac{_{d}D f(X)}{_d D X} &= \frac{_d D f(X)}{_f D f(X)} \frac{_f D f(X)}{_f D X} \frac{_f D X}{_d D X}\\
&= \begin{bmatrix}
    I_3 & 0_3\\
    0_3 & I_3
\end{bmatrix}
\frac{_f D f(X)}{_f D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times R\\
    0_3 & I_3
\end{bmatrix}\\
&=\frac{_f D f(X)}{_f D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times R\\
    0_3 & I_3
\end{bmatrix},
    
\end{aligned}
$$

where,

- $\frac{_{d}D f(X)}{_d D X}$ is the jacobian used in the basalt defined by decoupled left increment and decoupled log,
$$\frac{_{d}D f(X)}{_d D X} := \lim_{\tau \rightarrow 0} \frac{\log_d(f(X \oplus_d \tau)f(X)^{-1})}{\tau},$$ 

- $\frac{_d D f(X)}{_f D f(X)}$ is the jacobian of identity function defined by full SE3 left increment and decoupled log,
$$\frac{_d D f(X)}{_f D f(X)} := \lim_{\tau \rightarrow 0} \frac{\log_d((f(X) \oplus_f \tau) f(X)^{-1})}{\tau}
=I_6,$$

- $\frac{_f D f(X)}{_f D X}$ is the full SE3 jacobian of $f(X)$ w.r.t. $X$,
$$\frac{_f D f(X)}{_f D X} := \lim_{\tau \rightarrow 0} \frac{\log_f (f(X \oplus_f \tau) f(X)^{-1}) }{\tau},$$

- $\frac{_f D X}{_d D X}$ is the jacobian of the identity function defined by decoupled left increment and full SE3 log,
$$\frac{_f D X}{_d D X} := \lim_{\tau \rightarrow 0} \frac{\log_f ((X \oplus_d \tau) X^{-1})}{\tau}
=\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times\\
    0_3 & I_3
\end{bmatrix}
,$$

- 演算子、$\tau, X$の定義：
$$
\begin{aligned}
\log_d(X) &:= \begin{bmatrix}
    \boldsymbol{t}\\
    \log(R)
\end{bmatrix},\\
\log_f(X) &:= \begin{bmatrix}
    V(\log(R))^{-1} \boldsymbol{t}\\
    \log(R)
\end{bmatrix},\\
X \oplus_d \tau &:= \begin{bmatrix}
    \exp(\boldsymbol{\omega}) R & \boldsymbol{v} + \boldsymbol{t}\\
    0_{13} & 1    
\end{bmatrix},\\
X \oplus_f \tau &:= \begin{bmatrix}
    \exp(\boldsymbol{\omega}) & V(\omega) \boldsymbol{v}\\
    0_{13} & 1
\end{bmatrix} \begin{bmatrix}
    R & \boldsymbol{t}\\
    0_{13} & 1
\end{bmatrix},\\
X &= \begin{bmatrix}
    R & \boldsymbol{t}\\
    0_{13} & 1
\end{bmatrix},\\
\tau &= \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix}

\end{aligned}
$$

関数$V$, $V^{-1}$は、文献１の式（）です。

incrementやLogの定義の違いに戸惑いましたが、JacobianやAdjointについて考える良い機会にありました。ありがとうございます！




### Pose

I'm sorry for the late reply and also thank you for very helpful information.


I'm sorry for the late reply and also Thank you for the very helpful information.

#### Notations:
Notations used in my derivation are taken from the following paper.
Solà J, Deray J, Atchuthan D. A micro Lie theory for state estimation in robotics. Available from: http://arxiv.org/abs/1812.01537.

"Full SE3 Left jacobian of f(X) w.r.t. X." is defined as follows,
$$
\frac{^{\mathcal{E}}D f(X)}{D X} = 
\lim_{\boldsymbol{^{\mathcal{E}}\tau}\rightarrow 0} 
\frac{f(^{\mathcal{E}}\tau \oplus X)\ominus f(X)}
{^{\mathcal{E}}\tau},
$$
where

- $\mathcal{E} = \begin{bmatrix}
    I_3 & \boldsymbol{0}_{31}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)$ : Origin of SE3.
- $X=\begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)$
- $^{\mathcal{E}}\boldsymbol{\tau} = \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix} \in T_\mathcal{E} SE(3)$ : The vector representation of $\boldsymbol{\tau}\hat{} \in \mathfrak{se}3$
- $^{\mathcal{E}}\tau \oplus X = \begin{bmatrix}
    \mathrm{Exp}(\boldsymbol{\omega}) & \boldsymbol{V}(\boldsymbol{\omega})\boldsymbol{v} \\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}
\begin{bmatrix}
R & \boldsymbol{t}\\
\boldsymbol{0}_{13} & 1
\end{bmatrix}
$ : Full left increment of SE3 ([1] eq. 27, eq. 172)
- $A \ominus B = \mathrm{Log}(AB^{-1})$, $A, B \in SE(3)$ : from [1] eq.28
- $\mathrm{Log}(X) = \begin{bmatrix}
    \boldsymbol{V}(\boldsymbol{w})^{-1} \boldsymbol{t}\\
    Log_{SO3}(R)
\end{bmatrix}$ : from [1] eq.173


#### Decoupled left increment:
Thank you for the information on the derivation of jacobian and even the use of the term "decoupled left increment".
I realized that my last post was wrong.
It took me a while to understand those different Jacobians can be defined by different infinitesimal increments (Full or Decoupled). 
It was a good opportunity to learn  SE3 and Jacobian.  Thank you!



### Post
I'm sorry for the late reply and thank you for the very helpful information!

#### Notations:
Notations used in my derivation are taken from the following paper.

[1] Solà J, Deray J, Atchuthan D. A micro Lie theory for state estimation in robotics. Available from: http://arxiv.org/abs/1812.01537.

"Full SE3 Left jacobian of f(X) w.r.t. X." is defined as follows,
```math
\frac{^{\mathcal{E}}D f(X)}{D X} = 
\lim_{\boldsymbol{^{\mathcal{E}}\tau}\rightarrow 0} 
\frac{f(^{\mathcal{E}}\tau \oplus X)\ominus f(X)}
{^{\mathcal{E}}\tau},
```
where

- $`\mathcal{E} = \begin{bmatrix}
    I_3 & \boldsymbol{0}_{31}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)`$ : Origin of SE3.
- $`X=\begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\in SE(3)`$
- $`^{\mathcal{E}}\boldsymbol{\tau} = \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix} \in T_\mathcal{E} SE(3)`$ : The vector representation of $`\boldsymbol{\tau}\hat{} \in \mathfrak{se}3`$
- $`^{\mathcal{E}}\tau \oplus X = \begin{bmatrix}
    \mathrm{Exp}(\boldsymbol{\omega}) & \boldsymbol{V}(\boldsymbol{\omega})\boldsymbol{v} \\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}
\begin{bmatrix}
R & \boldsymbol{t}\\
\boldsymbol{0}_{13} & 1
\end{bmatrix}
`$ : Full left increment of SE3 ([1] eq. 27, eq. 172)
- $`A \ominus B = \mathrm{Log}(AB^{-1})`$, $`A, B \in SE(3)`$ : from [1] eq.28
- $`\mathrm{Log}(X) = \begin{bmatrix}
    \boldsymbol{V}(\boldsymbol{w})^{-1} \boldsymbol{t}\\
    Log_{SO3}(R)
\end{bmatrix}`$ : from [1] eq.173


#### Decoupled left increment:
Thank you for the derivation of jacobian and information of the term "decoupled left increment".
Thanks to your derivation, it makes sense to me why $\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times\\
    0_3 & I_3
\end{bmatrix}$
is used.
In my understanding, 
the conversion from the full SE3 left jacobian to the jacobian used in the basalt is derivated by the chain rule as follows, 

$$
\begin{aligned}
\frac{_{d}D f(X)}{_d D X} &= \frac{_d D f(X)}{_f D f(X)} \frac{_f D f(X)}{_f D X} \frac{_f D X}{_d D X}\\
&= \begin{bmatrix}
    I_3 & 0_3\\
    0_3 & I_3
\end{bmatrix}
\frac{_f D f(X)}{_f D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times R\\
    0_3 & I_3
\end{bmatrix}\\
&=\frac{_f D f(X)}{_f D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times R\\
    0_3 & I_3
\end{bmatrix},
    
\end{aligned}
$$

where,

- $\frac{_{d}D f(X)}{_d D X}$ is the jacobian used in the basalt defined by decoupled left increment and decoupled log,
$$\frac{_{d}D f(X)}{_d D X} := \lim_{\tau \rightarrow 0} \frac{\log_d(f(X \oplus_d \tau)f(X)^{-1})}{\tau},$$ 

- $\frac{_d D f(X)}{_f D f(X)}$ is the jacobian of identity function defined by full SE3 left increment and decoupled log,
$$\frac{_d D f(X)}{_f D f(X)} := \lim_{\tau \rightarrow 0} \frac{\log_d((f(X) \oplus_f \tau) f(X)^{-1})}{\tau}
=I_6,$$

- $\frac{_f D f(X)}{_f D X}$ is the full SE3 jacobian of $f(X)$ w.r.t. $X$,
$$\frac{_f D f(X)}{_f D X} := \lim_{\tau \rightarrow 0} \frac{\log_f (f(X \oplus_f \tau) f(X)^{-1}) }{\tau},$$

- $\frac{_f D X}{_d D X}$ is the jacobian of the identity function defined by decoupled left increment and full SE3 log,
$$\frac{_f D X}{_d D X} := \lim_{\tau \rightarrow 0} \frac{\log_f ((X \oplus_d \tau) X^{-1})}{\tau}
=\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times\\
    0_3 & I_3
\end{bmatrix}
,$$

- Definition for operators, $\tau, X$：
$$
\begin{aligned}
\log_d(X) &:= \begin{bmatrix}
    \boldsymbol{t}\\
    \log(R)
\end{bmatrix},\\
\log_f(X) &:= \begin{bmatrix}
    V(\log(R))^{-1} \boldsymbol{t}\\
    \log(R)
\end{bmatrix},\\
X \oplus_d \tau &:= \begin{bmatrix}
    \exp(\boldsymbol{\omega}) R & \boldsymbol{v} + \boldsymbol{t}\\
    0_{13} & 1    
\end{bmatrix},\\
X \oplus_f \tau &:= \begin{bmatrix}
    \exp(\boldsymbol{\omega}) & V(\omega) \boldsymbol{v}\\
    0_{13} & 1
\end{bmatrix} \begin{bmatrix}
    R & \boldsymbol{t}\\
    0_{13} & 1
\end{bmatrix},\\
X &= \begin{bmatrix}
    R & \boldsymbol{t}\\
    0_{13} & 1
\end{bmatrix},\\
\tau &= \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix}

\end{aligned}
$$

The function $V$ is eq.174 in [1].
I was initially confused by the differences in the definitions of increment and log, but it was a good opportunity to think about jacobian and adjoint of SE3. Thank you!
