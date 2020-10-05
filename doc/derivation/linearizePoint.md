# linearizePoint

## ToC
- res
- d_res_d_xi
- d_res_d_p
- $T \boldsymbol{q}$のJacobianの導出
- 便利式


## res
ベースとなるReprojection errorの式。論文と右辺の式のマイナスする順序が逆になっているので注意。
$$
\mathrm{res} = \boldsymbol{r}_{it} = 
\pi ( T_{W C_t}^{-1} T_{W C_h} \boldsymbol{q}({}_h \boldsymbol{m}_i)) - \boldsymbol{z}_{it}\\
= \pi (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))
$$

このとき、
- $\pi$ : Projection関数
- $\boldsymbol{q}(u,v,d) = [x,y,z,d^{-1}]^T = [\boldsymbol{p}, d^{-1}]^T$ : Stereographic projection関数
- ${}_h \boldsymbol{m} = [u,v,d]^T$ : Minimal representation.平面場位置$u,v$とその点までの距離d(HostFrame座標系)によってランドマーク位置を表現している。

また、
$$
T_{th} = 
\begin{bmatrix}
R_{th} & \boldsymbol{t}_{th}\\
\boldsymbol{0}_{13} & 1
\end{bmatrix} \\
\boldsymbol{q} = 
\begin{bmatrix}
\boldsymbol{p}\\
d^{-1}
\end{bmatrix}
$$


## d_res_d_xi

再投影誤差の $T_{th}$ についてのJacobian。
$\mathcal{E}$ における接空間(Lie algebra)で表現しているので注意。
$$ \frac{{}^{\mathcal{E}}D \boldsymbol{r}_{it}}{D T_{th}} = 
J_p
\begin{bmatrix}
d^{-1} I_{33} & - [R_{th} \boldsymbol{p} + d^{-1} \boldsymbol{t}_{th}]_{\times} \\
\boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$

### 導出
Reprojection errorの式から、
$$
\frac{{}^{\mathcal{E}}D \boldsymbol{r}_{it}}{D T_{th}} 
= 
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))}
\frac{{}^{\mathcal{E}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
= 
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))}
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)}
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
\boldsymbol{Ad}_{T_{th}^{-1}}
$$
が成り立つ。個々の要素を求めていくことで`d_res_d_xi`が計算できる。

まず、Projection関数のJacobianを$J_p$とおく。これはProjection関数（カメラモデル）が変わると内容が変化する。
$$
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))} = J_p
$$

次に、SE3($T_{th}$)と$\boldsymbol{q}$間のaction($T_{th}\boldsymbol{q}$)についてJacobianを計算する。ここでは、$T_{th}$の接空間におけるJacobianを求め、その後$\mathcal{E}$の接空間におけるJacobianに変換する。
まず、$T_{th}$の接空間におけるJacobianは、
$$
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}=
\begin{bmatrix}
    d^{-1}R_{th} & -R_{th}[\boldsymbol{p}]_\times\\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$
となる。$\boldsymbol{q}$の第４要素はInverse distance($d^{-1}$)なので、通常のSE3におけるGroup actionとは異なっていることに注意。
変換するためのAdjointは以下の通り。
$$
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)} = I_{33} \\
\boldsymbol{Ad}_{T_{th}^{-1}} =
\begin{bmatrix}
    R_{th}^T & [-R_{th}^T \boldsymbol{t}_{th}]_\times R_{th}^T\\
    \boldsymbol{0}_{33} & R_{th}^T
\end{bmatrix}
$$

よって、$\mathcal{E}$の接空間におけるJacobianは以下の通り。
$$
\frac{{}^{\mathcal{E}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}=
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)}
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
\boldsymbol{Ad}_{T_{th}^{-1}}
=
I_{33}
\begin{bmatrix}
    d^{-1}R_{th} & -R_{th}[\boldsymbol{p}]_\times\\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
\begin{bmatrix}
    R_{th}^T & [-R_{th}^T \boldsymbol{t}_{th}]_\times R_{th}^T\\
    \boldsymbol{0}_{33} & R_{th}^T
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} R_{th} R_{th}^T & d^{-1} R_{th}[-R_{th}^{T} \boldsymbol{t}_{th}]_\times R_{th}^T - R_{th}[\boldsymbol{p}]_\times R_{th}^T \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} I_{33} & - R_{th}[d^{-1} R_{th}^T \boldsymbol{t}_{th} + \boldsymbol{p} ]_\times R_{th}^T \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} I_{33} & - [d^{-1} \boldsymbol{t}_{th} + R_{th} \boldsymbol{p} ]_\times \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$


## d_res_d_p
再投影誤差の$\boldsymbol{m}_i \in \mathbb{R}^4$についてのJacobian。$\mathbb{R}^4$についてのJacobianなので
接空間の違いについて考える必要はなし。以下のように計算される。
$$

\mathrm{d\_res\_d\_p} = 
\frac{D \boldsymbol{r}_{it}}{D {}_h \boldsymbol{m}_i} = 
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))}
\frac{D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D \boldsymbol{q}({}_h\boldsymbol{m}_i)}
\frac{\partial \boldsymbol{q}({}_h\boldsymbol{m}_i)}{\partial {}_h\boldsymbol{m}_i}\\
= J_p T_{th} J_{up}

$$



## $T \boldsymbol{q}$のJacobian
### $T$についてのJacobian
$T \in SE(3)$と、$\boldsymbol{q} = [\boldsymbol{p}, d]^T$のAction:$T \boldsymbol{q}$について、
$$
\frac{{}^{T} D T \boldsymbol{q}}{D T}
$$
を計算する。
このとき、
$$
T_{th} = 
\begin{bmatrix}
    R & \boldsymbol{t} \\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}\\
\boldsymbol{q} = 
\begin{bmatrix}
    \boldsymbol{p} \in \mathbb{R}^3\\
    d
\end{bmatrix}
$$
としている。

Right-jacobianの定義から計算していく。以下のように最終的に分子を$J\boldsymbol{\tau}$の行列×ベクトルの形に変形できれば、そのときの行列$J$がRight-jacobianになる。この形を目標に式変形する。
$$
\frac{{}^{T} D T \boldsymbol{q}}{D T} = 
\lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} \frac{(T \oplus \tau) \boldsymbol{q} \ominus T \boldsymbol{q}}{\tau}\\
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} \frac{J\boldsymbol{\tau}}{\tau} = J
$$
なお、$\boldsymbol{\tau}$は$T$の接空間におけるベクトルを成分表示したもので、
$$
\boldsymbol{\tau} = 
\begin{bmatrix}
    \boldsymbol{\rho} \in \mathbb{R}^3\\
    \boldsymbol{\theta} \in \mathbb{R}^3
\end{bmatrix}
$$
としている。$\boldsymbol{\rho}$は$T$の座標系における速度ベクトル、$\boldsymbol{\theta}$は$T$の座標系における角速度ベクトル相当する。

分子を変形して、$J\boldsymbol{\tau}$の形にすることでRight-jacobianを計算する。
$$
\lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} \frac{(T \oplus \tau) \boldsymbol{q} \ominus T \boldsymbol{q}}{\tau}
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} \frac{T \mathrm{Exp}(\boldsymbol{\tau})\boldsymbol{q} - T \boldsymbol{q}}{\boldsymbol{\tau}}\\
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} 
\frac{
    \begin{bmatrix}
        R & \boldsymbol{t}\\
        \boldsymbol{0}_{13} & 1
    \end{bmatrix}
    \begin{bmatrix}
        \mathrm{Exp}(\boldsymbol{\theta}) & \boldsymbol{V}(\boldsymbol{\theta})\boldsymbol{\rho} \\
        \boldsymbol{0}_{13} & 1
    \end{bmatrix}
    \begin{bmatrix}
        \boldsymbol{p}\\
        d
    \end{bmatrix}
     - 
     \begin{bmatrix}
        R & \boldsymbol{t}\\
        \boldsymbol{0}_{13} & 1 
     \end{bmatrix} 
     \begin{bmatrix}
         \boldsymbol{p}\\
         d
     \end{bmatrix}
     }
     {\boldsymbol{\tau}}\\
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        R(\mathrm{Exp}(\boldsymbol{\theta})\boldsymbol{p} - \boldsymbol{p})+dR\boldsymbol{V}(\boldsymbol{\theta})\boldsymbol{\rho}
    \end{bmatrix}
}
{\boldsymbol{\tau}}
$$


このとき、$\tau \rightarrow \boldsymbol{0}$なので次の式が成り立つ。
$\sin$、$\cos$をテイラー展開し1次の項まで利用すれば、
$$
\mathrm{Exp}(\boldsymbol{\theta}) = \mathrm{Exp}(\theta \boldsymbol{u}) = I + \sin\theta [\boldsymbol{u}]_\times +(1-\cos\theta)[\boldsymbol{u}]_\times^2\\
\approx I + [\boldsymbol\theta]_\times
$$
また、
$$
\boldsymbol{V}(\theta) = I + \frac{1-\cos\theta}{\theta}[\boldsymbol{u}]_\times + \frac{\theta - \sin\theta}{\theta}[\boldsymbol{u}]_\times^2\\
\approx I
$$

よって、
$$
\lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}} \frac{(T \oplus \tau) \boldsymbol{q} \ominus T \boldsymbol{q}}{\tau}
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        R(\mathrm{Exp}(\boldsymbol{\theta})\boldsymbol{p} - \boldsymbol{p})+dR\boldsymbol{V}(\boldsymbol{\theta})\boldsymbol{\rho}\\
        0
    \end{bmatrix}
}
{\boldsymbol{\tau}}\\
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        R[\boldsymbol{\theta}]_\times \boldsymbol{p} + dRI\boldsymbol{\rho}\\
        0
    \end{bmatrix}
}{\boldsymbol{\tau}} 
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        -R[\boldsymbol{p}]_\times \boldsymbol{\theta} + dR\boldsymbol{\rho}\\
        0
    \end{bmatrix}
}{\boldsymbol{\tau}} 
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        dR & -R[\boldsymbol{p}]_\times\\
        \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
    \end{bmatrix}
    \begin{bmatrix}
        \boldsymbol{\rho}\\
        \boldsymbol{\theta}
    \end{bmatrix}
}{\boldsymbol{\tau}} \\
= \lim_{\boldsymbol{\tau}\rightarrow \boldsymbol{0}}
\frac{
    \begin{bmatrix}
        dR & -R[\boldsymbol{p}]_\times\\
        \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
    \end{bmatrix} \boldsymbol{\tau}
}{\boldsymbol{\tau}} \\
$$
つまり、
$$
\frac{{}^{T} D T \boldsymbol{q}}{D T} = 
\begin{bmatrix}
        dR & -R[\boldsymbol{p}]_\times\\
        \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
    \end{bmatrix}
$$

### $\boldsymbol{q}$について
$T$についての場合と同様に計算できる。

## 便利式
$R\in SO3$、$\boldsymbol{\theta} \in \mathbb{R}^3$として、
$$
[R \boldsymbol{\theta}]_\times = R [\boldsymbol{\theta}]_\times R^T
$$
また、
$$
[\boldsymbol{a}]_\times \boldsymbol{b} = - [\boldsymbol{b}]_\times \boldsymbol{a}
$$