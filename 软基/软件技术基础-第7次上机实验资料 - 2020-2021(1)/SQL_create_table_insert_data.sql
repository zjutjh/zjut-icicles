USE JXGL7
GO

create table SS
(
SCODE# char(10) primary key,
SSNAME nchar(10)
);

insert into SS values('S0401', '计算机科学与技术');
insert into SS values('S0402', '指挥自动化');
insert into SS values('S0403', '网络工程');
insert into SS values('S0404', '信息研究与安全');

create table C 
(
    C# char(10) PRIMARY KEY,
    CNAME nchar(10),
    CLASSH tinyint
);

insert into C values ('C401001', '数据结构', 70);
insert into C values ('C401002', '操作系统', 60);
insert into C values ('C402001', '计算机原理', 60);
insert into C values ('C402002', '通信原理', 60);
insert into C values ('C403001', '计算机网络', 60);
insert into C values ('C403002', '信息安全技术', 50);
insert into C values ('C404001', '信息编码与加密', 60);



create table S 
(
    S# char(10) PRIMARY KEY,
    SNAME nchar(10) NOT NULL,
    SSEX nchar(1) check(SSEX IN ('男','女')) NOT NULL,
    SBIRTHIN datetime,
    PLACEOFB nchar(20),
    SCODE# char(10) NOT NULL,
    CLASS nchar(10) NOT NULL,
    foreign key (SCODE#) references SS(SCODE#)
);


insert into S values ('200401001', '张华', '男', '1982-11-14', '北京', 'S0401', '200401');
insert into S values ('200401002', '李建平', '男', '1982-8-20', '上海', 'S0401', '200401');
insert into S values ('200401003', '王丽丽', '女', '1982-2-2', '上海', 'S0401', '200401');
insert into S values ('200402001', '杨秋红', '女', '1983-5-9', '西安', 'S0402', '200402');
insert into S values ('200402002', '吴志伟', '男', '1982-6-30', '南京', 'S0402', '200402');
insert into S values ('200402003', '李涛', '男', '1983-6-25', '西安', 'S0402', '200402');
insert into S values ('200403001', '赵晓艳', '女', '1982-3-11', '长沙', 'S0403', '200403');


create table SC 
(
    S# char(10),
    C# char(10),
    GRADE smallint CHECK(GRADE BETWEEN 0 AND 100),
    PRIMARY KEY(S#,C#),
    foreign key (S#) references S(S#),
    foreign key (C#) references C(C#)
);

insert into SC values ('200401001', 'C401001', 90);
insert into SC values ('200401001', 'C402002', 90);
insert into SC values ('200401001', 'C403001', null);
insert into SC values ('200401002', 'C401001', 75);
insert into SC values ('200401002', 'C402002', 88);
insert into SC values ('200401003', 'C402002', 69);
insert into SC values ('200402001', 'C401001', 87);
insert into SC values ('200402001', 'C401002', 90);
insert into SC values ('200402002', 'C403001', 92);
insert into SC values ('200402003', 'C403001', 83);
insert into SC values ('200403001', 'C403002', 91);

create table T (T# char(10) primary key, 
                TNAME nchar(5) not null, 
                TSEX nchar(1) check(TSEX IN ('男','女')), 
                TBIRTHIN datetime not null, 
                TITLEOF nchar(5), 
                TRSECTION nchar(10) not null, 
                TEL char(16));
                
insert into T values('T0401001','张国庆','男','1950-5-1','教授','计算机','8810801');
insert into T values('T0401002','徐浩','男','1977-6-22','讲师','计算机','8899202');
insert into T values('T0402001','张明敏','女','1962-8-30','教授','指挥自动化','8851803');
insert into T values('T0402002','李阳洋','女','1968-12-11','副教授','指挥自动化','8882604');
insert into T values('T0403001','郭宏伟','男','1959-11-29','副教授','网络工程','8815805');
insert into T values('T0403002','宋歌','女','1982-3-15',null,'网络工程',null);

create table TEACH
(
T# char(10),
C# char(10),
primary key(T#, C#),
foreign key (T#) references T(T#),
foreign key (C#) references C(C#)
);

insert into TEACH values('T0401001', 'C401002');
insert into TEACH values('T0401002', 'C401001');
insert into TEACH values('T0402001', 'C402001');
insert into TEACH values('T0402002', 'C402002');
insert into TEACH values('T0403002', 'C403001');



select top 3 * from S;
select top 3 * from C;
select top 3 * from SC;
select top 3 * from T;
select top 3 * from SS;