function exportCoeff(nomFiltre,num,den)
// Exportation de des coefficients d'un filtre vers un fichier
// d'entête.
// Usage : exportCoeff(nomFiltre,coeffNum,coeffDen)

// Ecriture Coefficients dans le fichier
file_id = mopen(sprintf("filtre%s.h",nomFiltre),'w');

fprintf(file_id,'#define NL_%s %i\n',nomFiltre,length(num));
fprintf(file_id,'FLOAT_T num%s[%i]={\n',nomFiltre,length(num));
for i=1:length(num)-1
  fprintf(file_id,'%10.40f,\n',num(i));
end
fprintf(file_id,'%10.40f};\n',num($));

fprintf(file_id,'#define DL_%s %i\n',nomFiltre,length(den));
fprintf(file_id,'FLOAT_T den%s[%i]={\n',nomFiltre,length(den));
for i=1:length(den)-1
  fprintf(file_id,'%10.40f,\n',den(i));
end
fprintf(file_id,'%10.40f};\n',den($));

mclose(file_id);

endfunction
