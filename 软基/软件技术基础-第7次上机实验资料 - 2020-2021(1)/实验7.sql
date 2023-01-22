----1
--update SC
--	set GRADE = 90
--where C# = 'C403001' and S# = 200402003

----2
--select S#,Sname,PLACEOFB FROM S
--	where Sname like '李%'

----3
--select AVG(SC.GRADE) AS AVE_GRADE,SC.S#,S.SNAME
--FROM SC,S
--WHERE SC.S# = S.S#
--GROUP BY SC.S#,SNAME
--HAVING COUNT(*)>=2 AND AVG(GRADE)>=85;

----4
--SELECT TNAME 
--	FROM T
--	where T.T# not in
--	(select t#
--	from Teach)

----5
--CREATE VIEW V_AverageGrade
--AS SELECT S.S#,S.SNAME,AVG(SC.GRADE) AS AVG_GRADE
--FROM S,SC
--WHERE S.S# = SC.S#
--GROUP BY S.S#,SNAME

----6
--SELECT  *
--FROM V_AverageGrade
--WHERE AVG_GRADE >85
--ORDER BY AVG_GRADE DESC

----7
--CREATE VIEW V_CourseGrade
--AS SELECT S.S#,S.SNAME,SC.GRADE,S.CLASS,C.CNAME,SC.C#
--FROM S,SC,C
--WHERE S.S# = SC.S# AND C.C# = SC.C#
--GROUP BY S.S#,SNAME,SC.GRADE,S.CLASS,C.CNAME,SC.C#

----8
--select grade,CNAME
--from V_CourseGrade
--where Sname = '李建平'
--order by grade desc