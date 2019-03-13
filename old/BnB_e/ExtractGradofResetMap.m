%%Extracting grading of a reset map... SS
syms ml mq v2i v1i e 
v2f=((ml*v2i+mq*v1i)-mq*e*(v2i-v1i))/(mq+ml);
v1f=v2f+e*(v2i-v1i);
jacobian([v2f;v1f],[v1i;v2i])%Under refinement