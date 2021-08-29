function T1Eq = GravityCompT1(Fdx,Fdy,I1,I2,I3,d1,d2,d3,g,l1,l2,l3,m1,m2,m3,th1,th2,th3,thdot1,thdot2,thdot3)
%GRAVITYCOMPT1
%    T1EQ = GRAVITYCOMPT1(FDX,FDY,I1,I2,I3,D1,D2,D3,G,L1,L2,L3,M1,M2,M3,TH1,TH2,TH3,THDOT1,THDOT2,THDOT3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Jan-2021 11:27:48

t2 = conj(Fdx);
t3 = conj(Fdy);
t4 = conj(th1);
t5 = conj(th2);
t6 = conj(th3);
t9 = d1.^2;
t10 = d2.^2;
t11 = d3.^2;
t7 = cos(t4);
t8 = sin(t4);
t13 = t4+t5;
t12 = l1.*t7;
t14 = l1.*t8;
t15 = t7.^2;
t16 = t8.^2;
t17 = cos(t13);
t19 = sin(t13);
t21 = d1.*g.*m1.*t7;
t22 = t6+t13;
t18 = t12.*thdot1;
t20 = t14.*thdot1;
t23 = d2.*t17;
t24 = l2.*t17;
t25 = d2.*t19;
t26 = cos(t22);
t27 = l2.*t19;
t28 = sin(t22);
t29 = t17.^2;
t30 = t19.^2;
t40 = m1.*t9.*t15;
t41 = m1.*t9.*t16;
t31 = t23.*thdot2;
t32 = t24.*thdot2;
t33 = t25.*thdot2;
t34 = t27.*thdot2;
t35 = g.*m2.*t23;
t36 = d3.*t26;
t37 = l3.*t26;
t38 = d3.*t28;
t39 = l3.*t28;
t42 = t26.^2;
t43 = t28.^2;
t49 = m2.*t12.*t23;
t50 = m2.*t10.*t29;
t51 = m2.*t14.*t25;
t52 = m2.*t10.*t30;
t53 = t12+t23;
t54 = t14+t25;
t44 = t36.*thdot3;
t45 = t38.*thdot3;
t46 = t3.*t37;
t47 = t2.*t39;
t48 = g.*m3.*t36;
t56 = m3.*t12.*t36;
t57 = m3.*t11.*t42;
t58 = m3.*t14.*t38;
t59 = m3.*t11.*t43;
t60 = t18+t31;
t61 = t20+t33;
t62 = g.*m2.*t53;
t63 = m3.*t24.*t36;
t64 = m3.*t27.*t38;
t65 = t24+t36;
t66 = t24+t37;
t67 = t27+t38;
t68 = t27+t39;
t69 = m2.*t12.*t53;
t70 = m2.*t14.*t54;
t74 = m2.*t23.*t53;
t75 = m2.*t25.*t54;
t55 = -t46;
t71 = t3.*t66;
t72 = t2.*t68;
t73 = g.*m3.*t65;
t76 = t12+t65;
t77 = t12+t66;
t79 = t14+t67;
t80 = t14+t68;
t81 = m3.*t12.*t65;
t82 = m2.*t25.*t60;
t83 = m2.*t23.*t61;
t84 = m3.*t14.*t67;
t85 = t18+t32+t44;
t86 = t20+t34+t45;
t91 = m3.*t24.*t65;
t92 = m3.*t27.*t67;
t95 = m3.*t36.*t65;
t96 = m3.*t38.*t67;
t98 = I3+t56+t58;
t99 = m2.*t54.*t60;
t100 = m2.*t53.*t61;
t101 = I3+t57+t59;
t108 = I3+t63+t64;
t78 = -t71;
t87 = -t83;
t88 = t3.*t77;
t89 = g.*m3.*t76;
t90 = t2.*t80;
t94 = m3.*t12.*t76;
t97 = m3.*t14.*t79;
t102 = m3.*t24.*t76;
t103 = m3.*t27.*t79;
t104 = -t100;
t105 = m3.*t36.*t76;
t106 = 1.0./t98;
t107 = m3.*t38.*t79;
t109 = m3.*t38.*t85;
t110 = m3.*t36.*t86;
t112 = m3.*t67.*t85;
t113 = m3.*t65.*t86;
t115 = m3.*t79.*t85;
t116 = m3.*t76.*t86;
t118 = I3+t95+t96;
t123 = I2+I3+t49+t51+t81+t84;
t125 = I2+I3+t50+t52+t91+t92;
t93 = -t88;
t111 = -t110;
t114 = -t113;
t117 = -t116;
t119 = t101.*t106;
t121 = I3+t105+t107;
t122 = t106.*t108;
t124 = 1.0./t123;
t126 = I2+I3+t74+t75+t102+t103;
t128 = I1+I2+I3+t40+t41+t69+t70+t94+t97;
t120 = -t119;
t127 = t47+t48+t55+t109+t111;
t129 = 1.0./t128;
t130 = t118.*t124;
t134 = t124.*t125;
t136 = t35+t72+t73+t78+t82+t87+t112+t114;
t140 = t21+t62+t89+t90+t93+t99+t104+t115+t117;
t131 = -t130;
t132 = t106.*t127;
t135 = t121.*t129;
t137 = t126.*t129;
t141 = t124.*t136;
t146 = t129.*t140;
t133 = -t132;
t138 = -t137;
t139 = t120+t135;
t142 = -t141;
t145 = t131+t135;
t143 = t122+t138;
t147 = t134+t138;
t152 = t133+t146;
t154 = t142+t146;
t144 = 1.0./t143;
t148 = 1.0./t147;
t149 = t129.*t144;
t150 = t129.*t148;
t153 = -t144.*(t119-t135);
t155 = -t148.*(t130-t135);
t156 = t148.*(t130-t135);
t157 = t106.*t123.*t144.*t147;
t158 = -t144.*(t132-t146);
t159 = t144.*(t132-t146);
t161 = -t148.*(t141-t146);
t151 = -t150;
t162 = t153+t156;
t163 = -1.0./(t155+t144.*(t119-t135));
t166 = (t106.*t144)./(t155+t144.*(t119-t135));
t183 = t159+t161;
t212 = -t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t213 = t124.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t95.*t124.*t148)./(t155+t144.*(t119-t135))+(t96.*t124.*t148)./(t155+t144.*(t119-t135))+1.0);
t214 = -t129.*((I3.*t124.*t148)./(t155+t144.*(t119-t135))+(t105.*t124.*t148)./(t155+t144.*(t119-t135))+(t107.*t124.*t148)./(t155+t144.*(t119-t135)));
t160 = t149+t151;
t164 = t106.*t144.*t163;
t169 = t36.*t166;
t170 = t38.*t166;
t175 = t124.*t148.*t163;
t192 = t163.*t183;
t196 = (t36.*t183)./(t155+t144.*(t119-t135));
t207 = -t124.*(I3.*t166+t95.*t166+t96.*t166);
t208 = -t129.*(I3.*t166+t105.*t166+t107.*t166);
t209 = t129.*(I3.*t166+t105.*t166+t107.*t166);
t232 = t213+t214;
t165 = I3.*t164;
t167 = t36.*t164;
t168 = t38.*t164;
t171 = t95.*t164;
t172 = t96.*t164;
t173 = t105.*t164;
t174 = t107.*t164;
t176 = I3.*t175;
t177 = t36.*t175;
t178 = t38.*t175;
t179 = t95.*t175;
t180 = t96.*t175;
t181 = t105.*t175;
t182 = t107.*t175;
t184 = t160.*t163;
t193 = I3.*t192;
t194 = t36.*t192;
t195 = t38.*t192;
t198 = t86+t196;
t217 = -t124.*((I3.*t160)./(t155+t144.*(t119-t135))+(t95.*t160)./(t155+t144.*(t119-t135))+(t96.*t160)./(t155+t144.*(t119-t135)));
t218 = -t129.*((I3.*t160)./(t155+t144.*(t119-t135))+(t105.*t160)./(t155+t144.*(t119-t135))+(t107.*t160)./(t155+t144.*(t119-t135))+1.0);
t219 = t129.*((I3.*t160)./(t155+t144.*(t119-t135))+(t105.*t160)./(t155+t144.*(t119-t135))+(t107.*t160)./(t155+t144.*(t119-t135))+1.0);
t225 = t207+t209;
t226 = -t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166));
t233 = 1.0./t232;
t234 = t148.*t232;
t241 = -I3.*(t164+t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)));
t185 = I3.*t184;
t186 = t36.*t184;
t187 = t38.*t184;
t188 = t96.*t184;
t189 = t95.*t184;
t190 = t107.*t184;
t191 = t105.*t184;
t197 = t85+t195;
t200 = m3.*t65.*t198;
t203 = m3.*t76.*t198;
t205 = t165+t171+t172;
t206 = t165+t173+t174;
t210 = t176+t179+t180-1.0;
t211 = t176+t181+t182;
t227 = I2.*t226;
t228 = t24.*t226;
t229 = t27.*t226;
t230 = t74.*t226;
t231 = t75.*t226;
t235 = I2.*t234;
t236 = t24.*t234;
t237 = t27.*t234;
t238 = t74.*t234;
t239 = t75.*t234;
t240 = t166+t226;
t244 = -m3.*t76.*(t167+t24.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)));
t245 = -m3.*t79.*(t168+t27.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)));
t246 = t175+t234;
t252 = t217+t219;
t253 = -t148.*(t218+t124.*((I3.*t160)./(t155+t144.*(t119-t135))+(t95.*t160)./(t155+t144.*(t119-t135))+(t96.*t160)./(t155+t144.*(t119-t135))));
t255 = t148.*(t218+t124.*((I3.*t160)./(t155+t144.*(t119-t135))+(t95.*t160)./(t155+t144.*(t119-t135))+(t96.*t160)./(t155+t144.*(t119-t135))));
t285 = -t233.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166));
t199 = m3.*t67.*t197;
t201 = -t200;
t202 = m3.*t79.*t197;
t204 = -t203;
t215 = t185+t188+t189;
t216 = t185+t190+t191-1.0;
t242 = t169+t228;
t243 = t170+t229;
t247 = I3.*t246;
t248 = t177+t236;
t249 = t178+t237;
t254 = I2.*t253;
t256 = t24.*t253;
t257 = t27.*t253;
t258 = t24.*t255;
t259 = t27.*t255;
t260 = t75.*t253;
t261 = t74.*t253;
t262 = t184+t255;
t263 = -I3.*(t253+t160./(t155+t144.*(t119-t135)));
t264 = I3.*(t253+t160./(t155+t144.*(t119-t135)));
t293 = t227+t230+t231+t241+t244+t245;
t220 = t35+t72+t73+t78+t82+t87+t193+t199+t201;
t221 = t21+t62+t89+t90+t93+t99+t104+t193+t202+t204;
t250 = m3.*t76.*t248;
t251 = m3.*t79.*t249;
t265 = t186+t258;
t266 = t187+t259;
t222 = t124.*t220;
t224 = t129.*t221;
t267 = m3.*t76.*t265;
t268 = m3.*t79.*t266;
t294 = t235+t238+t239+t247+t250+t251;
t223 = -t222;
t269 = -t267;
t270 = -t268;
t272 = -t148.*(t222-t224);
t278 = t23.*t148.*(t222-t224);
t279 = t24.*t148.*(t222-t224);
t287 = -I3.*(t148.*(t222-t224)+t183./(t155+t144.*(t119-t135)));
t290 = -m3.*t79.*(-t85+t27.*t148.*(t222-t224)+(t38.*t183)./(t155+t144.*(t119-t135)));
t295 = 1.0./t294;
t271 = t223+t224;
t273 = I2.*t272;
t274 = t23.*t272;
t275 = t24.*t272;
t276 = t25.*t272;
t277 = t27.*t272;
t281 = t61+t278;
t286 = t192+t272;
t289 = t198+t279;
t296 = t254+t260+t261+t264+t269+t270+1.0;
t298 = -t295.*(I3.*(t164+t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+m3.*t76.*(t167+t24.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+m3.*t79.*(t168+t27.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+I2.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166))+t74.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166))+t75.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)));
t299 = t295.*(I3.*(t164+t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+m3.*t76.*(t167+t24.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+m3.*t79.*(t168+t27.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)))+I2.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166))+t74.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166))+t75.*t148.*(t208+t124.*(I3.*t166+t95.*t166+t96.*t166)));
t304 = -t295.*(t263+t267+t268+I2.*t255+t74.*t255+t75.*t255-1.0);
t280 = t60+t276;
t283 = m2.*t53.*t281;
t288 = t197+t277;
t291 = m3.*t76.*t289;
t300 = t157+t298;
t302 = t285+t299;
t282 = m2.*t54.*t280;
t284 = -t283;
t292 = -t291;
t301 = 1.0./t300;
t303 = 1.0./t302;
t297 = t21+t62+t89+t90+t93+t273+t282+t284+t287+t290+t292;
t305 = t295.*t297;
T1Eq = -(t303.*(t305+t233.*(t222-t224))+t301.*(t305-t123.*t147.*t183))./(t301.*(t295.*(t263+t267+t268+I2.*t255+t74.*t255+t75.*t255-1.0)-t123.*t147.*t160)+t303.*(t295.*(t263+t267+t268+I2.*t255+t74.*t255+t75.*t255-1.0)-t233.*(t218+t124.*((I3.*t160)./(t155+t144.*(t119-t135))+(t95.*t160)./(t155+t144.*(t119-t135))+(t96.*t160)./(t155+t144.*(t119-t135))))));
