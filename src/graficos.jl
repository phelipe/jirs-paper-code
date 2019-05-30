# Arquivo que contém as funções para plotar os gráficos das simulações





function plot_xv(t, xdata, xdata2, xdata3, xref, vdata, vdata2, vdata3,vref, joint )
    p1 = plot(t, xdata[joint] , label = "PD", title = "Posição Junta $(joint)", line=(2))
    p1 = plot!(t,xdata2[joint] , label = "OPD", line=(2,:dash))
    p1 = plot!(t,xdata3[joint] , label = "OFPD", line=(2,:dash))
    p1 = plot!(t, [xref[joint](j) for j in t], label = "Objetivo",line=(2, :dot))

    p2 = plot(t,vdata[joint], label = "PD", title = "Velocidade Junta $(joint)", line=(2))
    p2 = plot!(t,vdata2[joint] , label = "OPD", line=(2,:dash))
    p2 = plot!(t,vdata3[joint] , label = "OFPD", line=(2,:dash))
    p2 = plot!(t, [vref[joint](j) for j in t], label = "Objetivo",line=(2, :dot))
    
    plot(p1, p2)
end

function plot_aj(ta, tj, adata, adata2, adata3, aref, jdata,jdata2,jdata3,  jref, joint )
    p1 = plot(ta,adata[joint] , label = "PD", title = "Aceleração Junta $(joint)", line=(2))
    p1 = plot!(ta,adata2[joint], label = "OPD", line=(2,:dash))
    p1 = plot!(ta,adata3[joint], label = "OFPD", line=(2,:dash))
    p1 = plot!(ta, [aref[joint](j) for j in ta], label = "Objetivo",line=(2, :dot))
    
    p2 = plot(tj,jdata[joint], label = "PD", title = "Arrancada Junta $(joint)", line=(2))
    p2 = plot!(tj,jdata2[joint], label = "OPD", line=(2,:dash))
    p2 = plot!(tj,jdata3[joint], label = "OFPD", line=(2,:dash))
    p2 = plot!(tj, [jref[joint](j) for j in tj], label = "Objetivo",line=(2, :dot))
    
    plot(p1, p2)
end;


function relatorio(ex, exo, exf, ev, evo, evf, ea, eao,eaf, ej, ejo, ejf, joint)
    println("O erro final PD absoluto da posição para a junta $(joint) é:       $((abs(ex[joint][end])))") 
    println("O erro final OPD absoluto da posição para a junta $(joint) é:      $((abs(exo[joint][end])))") 
    println("---------------------------------------------------------------------------------------------")

    println("O iae PD  da posição para a junta $(joint) é:      $( iae((abs.(ex[joint]))) )")
    println("O iae OPD da posição para a junta $(joint) é:      $( iae((abs.(exo[joint]))) )")
    println("O iae OFPD da posição para a junta $(joint) é:     $( iae((abs.(exf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("O iae PD  da velocidade para a junta $(joint) é:      $( iae((abs.(ev[joint]))) )")
    println("O iae OPD da velocidade para a junta $(joint) é:      $( iae((abs.(evo[joint]))) )")
    println("O iae OFPD da velocidade para a junta $(joint) é:      $( iae((abs.(evf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("O iae PD  da aceleração para a junta $(joint) é:     $( iae((abs.(ea[joint]))) )")
    println("O iae OPD da aceleração para a junta $(joint) é:     $( iae((abs.(eao[joint]))) )")
    println("O iae OFPD da aceleração para a junta $(joint) é:     $( iae((abs.(eaf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("O iae PD   da arrancada para a junta $(joint) é:      $( iae((abs.(ej[joint]))) )")
    println("O iae OPD  da arrancada para a junta $(joint) é:      $( iae((abs.(ejo[joint]))) )")
    println("O iae OFPD  da arrancada para a junta $(joint) é:     $( iae((abs.(ejf[joint]))) )")
end