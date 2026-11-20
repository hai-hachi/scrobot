'use strict';
// Tracker full-SMC equations. Every expression has original LaTeX source
// and an offline-native MathML fallback (no CDN required for legible math).
// A non-blocking optional KaTeX enhancement provides typeset LaTeX when online.
(function(root){
  const tag=(name,body)=>'<'+name+'>'+body+'</'+name+'>';
  const mi=t=>tag('mi',t);
  const mo=t=>tag('mo',t.replaceAll('&','&amp;').replaceAll('<','&lt;'));
  const mn=t=>tag('mn',t);
  const row=(...values)=>tag('mrow',values.join(''));
  const sub=(base,suffix)=>'<msub>'+mi(base)+(isNaN(Number(suffix))?mi(suffix):mn(suffix))+'</msub>';
  const sup=(base,power)=>'<msup>'+base+mn(power)+'</msup>';
  const frac=(numerator,denominator)=>'<mfrac>'+row(numerator)+row(denominator)+'</mfrac>';
  const par=x=>row(mo('('),x,mo(')'));
  const fn=(name,arg)=>row('<mi mathvariant="normal">'+name+'</mi>',par(arg));
  const minus=mo('−'),plus=mo('+'),eq=mo('=');
  const EX=sub('e','x'), EY=sub('e','y'), ET=sub('e','θ');
  const LX=sub('λ','x'), LY=sub('λ','y');
  const KX=sub('k','x'), KS=sub('k','s');
  const ETAX=sub('η','x'), PHIX=sub('φ','x');
  const ETA=mi('η'), PHI=mi('φ');
  const WR=sub('ω','R'),VR=sub('v','R');
  const XR=sub('x','R'),YR=sub('y','R'),TR=sub('θ','R');
  const SR=sub('s','R'),S1=sub('s','1'),S2=sub('s','2');
  const g=arg=>row(sub('g','x'),par(arg));
  const q=arg=>row(mi('q'),par(arg));
  const sat=arg=>row('<mi mathvariant="normal">sat</mi>',par(arg));
  const v1=sub('v','1s'),w1=sub('ω','1s');
  const v2=sub('v','2s'),w2=sub('ω','2s');
  const gxDef=row(KX,EX,plus,sub('χ','sat'),ETAX,sat(frac(EX,PHIX)));
  const qDef=row(KS,mi('s'),plus,sub('χ','sat'),ETA,sat(frac(mi('s'),PHI)));
  const commonV=omega=>row(VR,fn('cos',ET),plus,EY,omega,plus,g(EX));
  const baseW=row(WR,plus,LY,VR,fn('sin',ET));
  const definitions={
    'body-x':{
      tex:'e_x=\\cos\\theta\\,(x_R-x)+\\sin\\theta\\,(y_R-y)',
      mml:row(EX,eq,fn('cos',mi('θ')),par(row(XR,minus,mi('x'))),plus,
              fn('sin',mi('θ')),par(row(YR,minus,mi('y'))))
    },
    'body-y':{
      tex:'e_y=-\\sin\\theta\\,(x_R-x)+\\cos\\theta\\,(y_R-y)',
      mml:row(EY,eq,minus,fn('sin',mi('θ')),par(row(XR,minus,mi('x'))),
              plus,fn('cos',mi('θ')),par(row(YR,minus,mi('y'))))
    },
    'body-heading':{
      tex:'e_\\theta=\\operatorname{wrap}(\\theta_R-\\theta)',
      mml:row(ET,eq,fn('wrap',row(TR,minus,mi('θ'))))
    },
    'carrot-radius':{
      tex:'(x_R-x)^2+(y_R-y)^2=L^2\\quad\\text{(PP-circle lookahead)}',
      mml:row(sup(par(row(XR,minus,mi('x'))),2),plus,
              sup(par(row(YR,minus,mi('y'))),2),eq,
              sup(mi('L'),2))
    },
    'carrot-window':{
      tex:'A=P(s_R-h),\\quad B=P(s_R),\\quad C=P(s_R+h)',
      mml:row(mi('A'),eq,fn('P',row(SR,minus,mi('h'))),mo(';'),
              mi('B'),eq,fn('P',SR),mo(';'),
              mi('C'),eq,fn('P',row(SR,plus,mi('h'))))
    },
    'carrot-curvature':{
      tex:'\\kappa_R=\\frac{2\\left[(B-A)\\times(C-A)\\right]}{\\lVert B-A\\rVert\\lVert C-B\\rVert\\lVert C-A\\rVert}',
      mml:row(sub('κ','R'),eq,frac(
        row(mn('2'),par(row(par(row(mi('B'),minus,mi('A'))),mo('×'),
          par(row(mi('C'),minus,mi('A')))))),
        row(mo('|'),mi('B'),minus,mi('A'),mo('|'),
            mo('|'),mi('C'),minus,mi('B'),mo('|'),
            mo('|'),mi('C'),minus,mi('A'),mo('|'))))
    },
    'reference-speed':{
      tex:'v_R=v_{\\mathrm{shared}}\\,r_{\\mathrm{speed}}\\min\\!\\left(1,\\frac{d_{\\mathrm{remain}}}{0.75\\,\\mathrm m}\\right)',
      mml:row(VR,eq,sub('v','shared'),sub('r','speed'),
        fn('min',row(mn('1'),mo(','),frac(sub('d','remain'),row(mn('0.75'),mi('m'))))))
    },
    'reference-omega':{
      tex:'\\omega_R=v_R\\,\\kappa_R',
      mml:row(WR,eq,VR,sub('κ','R'))
    },
    'denominator':{
      tex:'D=1+\\lambda_y e_x,\\qquad |D|>\\delta_D',
      mml:row(mi('D'),eq,mn('1'),plus,LY,EX,mo(';'),
        mo('|'),mi('D'),mo('|'),mo('>'),sub('δ','D'))
    },
    'switch-enable':{
      tex:'\\chi_{\\mathrm{sat}}=\\begin{cases}1 &\\text{boundary saturation enabled}\\\\0&\\text{disabled}\\end{cases}',
      mml:row(sub('χ','sat'),eq,mo('{'),
        '<mtable><mtr><mtd>'+mn('1')+'</mtd><mtd><mtext>enabled</mtext></mtd></mtr>'+
        '<mtr><mtd>'+mn('0')+'</mtd><mtd><mtext>disabled</mtext></mtd></mtr></mtable>')
    },
    'sat-definition':{
      tex:'\\operatorname{sat}(z)=\\begin{cases}-1,&z<-1\\\\z,&|z|\\le1\\\\+1,&z>1\\end{cases}',
      mml:row(sat(mi('z')),eq,mo('{'),
        '<mtable>'+
          '<mtr><mtd>'+minus+mn('1')+'</mtd><mtd>'+mi('z')+mo('<')+minus+mn('1')+'</mtd></mtr>'+
          '<mtr><mtd>'+mi('z')+'</mtd><mtd>'+mo('|')+mi('z')+mo('|')+mo('≤')+mn('1')+'</mtd></mtr>'+
          '<mtr><mtd>'+plus+mn('1')+'</mtd><mtd>'+mi('z')+mo('>')+mn('1')+'</mtd></mtr>'+
        '</mtable>')
    },
    'gx':{
      tex:'g_x(e_x)=k_x e_x+\\chi_{\\mathrm{sat}}\\eta_x\\operatorname{sat}\\!\\left(\\frac{e_x}{\\phi_x}\\right)',
      mml:row(g(EX),eq,gxDef)
    },
    'q':{
      tex:'q(s)=k_s s+\\chi_{\\mathrm{sat}}\\eta\\operatorname{sat}\\!\\left(\\frac{s}{\\phi}\\right)',
      mml:row(q(mi('s')),eq,qDef)
    },
    'smc1-s':{
      tex:'s=e_\\theta+\\lambda_y e_y+\\lambda_x e_x',
      mml:row(mi('s'),eq,ET,plus,LY,EY,plus,LX,EX)
    },
    'smc1-omega':{
      tex:'\\omega_{1\\mathrm s}=\\frac{\\omega_R+\\lambda_y v_R\\sin(e_\\theta)-\\lambda_x g_x(e_x)+q(s)}{1+\\lambda_y e_x}',
      mml:row(w1,eq,frac(row(baseW,minus,LX,g(EX),plus,q(mi('s'))),
                            row(mn('1'),plus,LY,EX)))
    },
    'smc1-v':{
      tex:'v_{1\\mathrm s}=v_R\\cos(e_\\theta)+e_y\\omega_{1\\mathrm s}+g_x(e_x)',
      mml:row(v1,eq,commonV(w1))
    },
    'smc1-reaching':{
      tex:'\\dot e_x=-g_x(e_x),\\qquad \\dot s=-q(s)\\quad\\text{(ideal only)}',
      mml:row('<mover>'+EX+mo('˙')+'</mover>',eq,minus,g(EX),mo(';'),
        '<mover>'+mi('s')+mo('˙')+'</mover>',eq,minus,q(mi('s')))
    },
    'smc2-s':{
      tex:'s_1=e_x,\\qquad s_2=e_\\theta+\\lambda_y e_y',
      mml:row(S1,eq,EX,mo(';'),S2,eq,ET,plus,LY,EY)
    },
    'smc2-omega':{
      tex:'\\omega_{2\\mathrm s}=\\frac{\\omega_R+\\lambda_y v_R\\sin(e_\\theta)+q(s_2)}{1+\\lambda_y e_x}',
      mml:row(w2,eq,frac(row(baseW,plus,q(S2)),row(mn('1'),plus,LY,EX)))
    },
    'smc2-v':{
      tex:'v_{2\\mathrm s}=v_R\\cos(e_\\theta)+e_y\\omega_{2\\mathrm s}+g_x(e_x)',
      mml:row(v2,eq,commonV(w2))
    },
    'smc2-reaching':{
      tex:'\\dot s_1=-g_x(e_x),\\qquad\\dot s_2=-q(s_2)\\quad\\text{(ideal only)}',
      mml:row('<mover>'+S1+mo('˙')+'</mover>',eq,minus,g(EX),mo(';'),
              '<mover>'+S2+mo('˙')+'</mover>',eq,minus,q(S2))
    },
    'limit-angular':{
      tex:'v_{\\mathrm{cmd}}\\ge0,\\qquad |\\omega_{\\mathrm{cmd}}|\\le\\omega_{\\max}',
      mml:row(sub('v','cmd'),mo('≥'),mn('0'),mo(';'),
        mo('|'),sub('ω','cmd'),mo('|'),mo('≤'),sub('ω','max'))
    },
    'limit-accel':{
      tex:'|\\Delta v|\\le a_{\\max}\\Delta t,\\qquad |\\Delta\\omega|\\le\\alpha_{\\max}\\Delta t',
      mml:row(mo('|'),mi('Δ'),mi('v'),mo('|'),mo('≤'),sub('a','max'),
        mi('Δ'),mi('t'),mo(';'),mo('|'),mi('Δ'),mi('ω'),mo('|'),mo('≤'),
        sub('α','max'),mi('Δ'),mi('t'))
    },
    'wheel-kinematics':{
      tex:'v_{L,w}=v-\\frac{b}{2}\\omega,\\qquad v_{R,w}=v+\\frac{b}{2}\\omega,\\qquad b=0.420\\;\\mathrm{m}',
      mml:row(sub('v','L,w'),eq,mi('v'),minus,
        frac(mi('b'),mn('2')),mi('ω'),mo(';'),
        sub('v','R,w'),eq,mi('v'),plus,
        frac(mi('b'),mn('2')),mi('ω'),mo(';'),
        mi('b'),eq,mn('0.420'),mi('m'))
    },
    'wheel-rpm':{
      tex:'v_{\\mathrm{wheel,max}}=\\min\\!\\left(0.50,\\frac{95\\cdot 2\\pi\\cdot0.050}{60}\\right)=0.49742\\;\\mathrm{m/s}',
      mml:row(sub('v','wheel,max'),eq,
        fn('min',row(mn('0.50'),mo(','),
          frac(row(mn('95'),mo('×'),mn('2'),mi('π'),mn('0.050')),
               mn('60')))),
        eq,mn('0.49742'),mi('m/s'))
    },
    'limit-wheel':{
      tex:'|v_{\\mathrm{cmd}}|+\\frac{0.420}{2}|\\omega_{\\mathrm{cmd}}|\\le0.49742\\;\\mathrm{m/s}',
      mml:row(mo('|'),sub('v','cmd'),mo('|'),plus,
        frac(mn('0.420'),mn('2')),
        mo('|'),sub('ω','cmd'),mo('|'),mo('≤'),mn('0.49742'),mi('m/s'))
    },
    'limit-spin':{
      tex:'|\\omega|_{\\max, v=0}=\\frac{2v_{\\mathrm{wheel,max}}}{b}=2.36866\\;\\mathrm{rad/s}',
      mml:row(mo('|'),mi('ω'),mo('|'),sub('ω','max'),
        eq,frac(row(mn('2'),sub('v','wheel,max')),mi('b')),
        eq,mn('2.36866'),mi('rad/s'))
    }
  };

  function render(useKatex=false){
    for(const el of document.querySelectorAll('[data-formula]')){
      const formula=definitions[el.dataset.formula];
      if(!formula)continue;
      el.dataset.latex=formula.tex;
      el.title='LaTeX: '+formula.tex;
      // HTML in the fallback is built only from static, trusted literals.
      if(useKatex&&root.katex){
        try{root.katex.render(formula.tex,el,{displayMode:true,
          throwOnError:true,strict:'ignore'});
          continue;
        }catch(_){ /* keep the fully local MathML renderer */ }
      }
      el.innerHTML='<math xmlns="http://www.w3.org/1998/Math/MathML" display="block">'+
        '<mstyle displaystyle="true">'+formula.mml+'</mstyle></math>';
    }
  }
  const keys=[
    ['lambda_y','λy'],['lambda_x','λx'],['k_s','ks'],['k_x','kx'],
    ['eta','η'],['phi','φ'],['eta_x','ηx'],['phi_x','φx'],
    ['singular_guard','δD'],['max_angular_cmd','ωmax'],
    ['max_linear_accel','amax'],['max_angular_accel','αmax'],
    ['use_saturation','saturation']
  ];
  function refresh(){
    for(const group of ['1','2']){
      const box=document.getElementById('smc'+group+'-math-values');
      if(!box)continue;
      box.replaceChildren();
      for(const [name,symbol] of keys){
        const input=document.getElementById('smc'+group+'-'+name.replaceAll('_','-'));
        if(!input)continue;
        const value=name==='use_saturation'?
          (input.checked?'on':'off'):input.value;
        const chip=document.createElement('span');
        chip.className='math-value-chip';
        chip.textContent=symbol+' = '+value;
        box.append(chip);
      }
    }
  }
  function init(){
    render(false);
    refresh();
    // Optional non-blocking enhancement. If disconnected, the native
    // MathML output remains visible and usable with no external dependency.
    if(!root.katex){
      const script=document.createElement('script');
      script.async=true;
      script.src='https://cdn.jsdelivr.net/npm/katex@0.16.22/dist/katex.min.js';
      script.onload=()=>{
        const css=document.createElement('link');
        css.rel='stylesheet';
        css.href='https://cdn.jsdelivr.net/npm/katex@0.16.22/dist/katex.min.css';
        document.head.append(css);
        css.onload=()=>render(true);
      };
      document.head.append(script);
    }else render(true);
  }
  if(typeof module!=='undefined'&&module.exports){
    module.exports={definitions};
  }
  if(root)root.TrackerMath={render,refresh,definitions};
  if(typeof document!=='undefined'){
    if(document.readyState==='loading')
      document.addEventListener('DOMContentLoaded',init,{once:true});
    else init();
  }
})(typeof window!=='undefined'?window:null);
