import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Properties;

public class J202203150619_4 extends JFrame {
    //****************************************************************************************************************//
    //成员变量
    //****************************************************************************************************************//
    private static String title = "202203150619－王晨阳－Java 程序设计综合实验";
    //      菜单栏相关
    private static JMenuBar mb = new JMenuBar();
    private static JMenu mFile = new MyMenu("文件(F)", KeyEvent.VK_F),
            mHomeWork = new MyMenu("Java上机题目"),
            mPhone = new MyMenu("通讯录(C)", KeyEvent.VK_C);
    private static JMenuItem miFileNew = new JMenuItem("新建(N)", KeyEvent.VK_N),
            miFileOpen = new JMenuItem("打开(O)...", KeyEvent.VK_O),
            miFileSave = new JMenuItem("保存(S)", KeyEvent.VK_S),
            miFileFont = new JMenuItem("字体与颜色(F)...", KeyEvent.VK_F),
            miFileBG = new JMenuItem("背景颜色(B)...", KeyEvent.VK_B),
            miFileQuit = new JMenuItem("退出(X)", KeyEvent.VK_X),
            miHWHuiwen = new JMenuItem("回文数"),
            miHWTrans = new JMenuItem("数字与英文互译"),
            miHWCompute = new JMenuItem("统计英文数据"),
            miHWPhoneCheck = new JMenuItem("手机号合法性判断"),
            miHWSum = new JMenuItem("文本文件求和"),
            miPhoneDisplay = new JMenuItem("通讯录维护"),
            miPhoneSave = new JMenuItem(" 通讯录储存文件设置");

    //      文本区相关
    private static JTextArea ta = new JTextArea();
    //字体与字体大小
    private static String font = "楷体";
    private static int fontSize = 15;
    private static String fontColor = "black";
    private static String bgColor = "#ffffff";
    //初始化为当前文件夹
    private static String filePath = System.getProperty("user.dir").replace("\\", "\\\\") + "\\\\",
            fileName = null;
    private static String textContent = "";
    //字体选择窗口
    private static MyFontDly myFontDly;

    {
        InitTextArea();
        myFontDly = new MyFontDly("字体");
        myFontDly.hideFontDly();
        myFontDly.setDefaultCloseOperation(DO_NOTHING_ON_CLOSE);
        myFontDly.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                myFontDly.hideFontDly();
            }
        });
    }

    //      通讯录相关
    //数据文件
    private static String dataFilePath = System.getProperty("user.dir").replace("\\", "\\\\") + "\\\\",
            dataFileName = "data.dat";

    private static ContactManager mana;

    //****************************************************************************************************************//
    //初始化
    //****************************************************************************************************************//
    private J202203150619_4(String title) {
        super(title);

        //添加菜单条 并 激活其使用
        addMenus();
        activeMenus();

        //读取通讯录数据储存位置
        readinDataPath();

        //重写setSize方法来自动适应不同分辨率
        setSize();
    }

    private void addMenus() {
        setJMenuBar(mb);

        //文件按钮添加选项
        mFile.add(miFileNew);
        mFile.add(miFileOpen);
        mFile.add(miFileSave);
        mFile.addSeparator();
        mFile.add(miFileFont);
        mFile.add(miFileBG);
        mFile.addSeparator();
        mFile.add(miFileQuit);

        mHomeWork.add(miHWHuiwen);
        mHomeWork.add(miHWTrans);
        mHomeWork.add(miHWCompute);
        mHomeWork.add(miHWPhoneCheck);
        mHomeWork.add(miHWSum);

        mPhone.add(miPhoneDisplay);
        mPhone.add(miPhoneSave);

        //把按钮添加进菜单条
        mb.add(mFile);
        mb.add(mHomeWork);
        mb.add(mPhone);
    }

    private void activeMenus() {
        //激活文件按钮

        //新建按钮
        miFileNew.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                newFile();
            }
        });
        //打开按钮
        miFileOpen.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                openFile();
            }
        });
        //保存按钮
        miFileSave.setAccelerator(KeyStroke.getKeyStroke(
                KeyEvent.VK_S, InputEvent.CTRL_DOWN_MASK
        ));
        miFileSave.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                saveFile();
            }
        });
        //字体与颜色按钮
        miFileFont.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                setFont();
            }
        });
        //背景颜色按钮
        miFileBG.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                setBGColor();
            }
        });
        //退出按钮
        miFileQuit.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                checkQuit();
            }
        });


        //激活java上机题目按钮
        //回文数
        miHWHuiwen.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                checkHuiWen();
            }
        });
        //数字与英文互译
        miHWTrans.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                transNumToEng();
            }
        });
        //统计英文数据
        miHWCompute.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                computeWords();
            }
        });
        //手机号码合法性判断
        miHWPhoneCheck.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                phoneCheck();
            }
        });
        //文本文件求和
        miHWSum.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                textSum();
            }
        });

        //通讯录按钮
        //通讯录维护
        miPhoneDisplay.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                try {
                    displayData();
                } catch (Exception ex) {
                    System.out.println("displayData error");
                    throw new RuntimeException(ex);
                }
            }
        });
        //通讯录储存文件设置
        miPhoneSave.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                setDataPath();
            }
        });
    }

    private void setSize() {
        //获得桌面大小
        Toolkit tk = getToolkit();
        Dimension dm = tk.getScreenSize();

        //初始宽为屏幕的 0.65倍， 高为屏幕的 0.65倍
        setSize((int) (dm.getWidth() * 0.65), (int) (dm.getHeight() * 0.65));
    }

    private void InitTextArea() {
        readinFont();
        //当前使用的字体与大小
        ta.setFont(new Font(font, Font.PLAIN, fontSize));

        ta.setForeground(stringToColor(fontColor));
        ta.setBackground(Color.decode(bgColor));

        //使文本区可滚动
        add(new JScrollPane(ta));
    }

    private void readinFont() {
        File fontConfig = new File("font.properties");
        if (!fontConfig.exists()) {
            try {
                fontConfig.createNewFile();
                BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("font.properties"), "gbk"));
                writer.write("font=楷体\nfontSize=15\nfontColor=black\nbgColor=#ffffff");
                writer.flush();
                writer.close();
            } catch (IOException e) {
                System.out.println("readinFont error");
                throw new RuntimeException(e);
            }
        }
        Properties fontProp = new Properties();
        try {
            InputStreamReader reader = new InputStreamReader(new FileInputStream(fontConfig), "gbk");
            fontProp.load(reader);
            font = fontProp.getProperty("font");
            fontSize = Integer.parseInt(fontProp.getProperty("fontSize"));
            fontColor = fontProp.getProperty("fontColor");
            bgColor = fontProp.getProperty("bgColor");
        } catch (IOException e) {
            System.out.println("readinFont error");
            throw new RuntimeException(e);
        }
    }

    private void readinDataPath() {
        File fontConfig = new File("datapath.properties");
        if (!fontConfig.exists()) {
            try {
                fontConfig.createNewFile();
                BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("datapath.properties"), "gbk"));
                writer.write(String.format("dataFilePath=%s\ndataFileName=%s", dataFilePath.replace("\\", "\\\\"), dataFileName));
                writer.flush();
                writer.close();
            } catch (IOException e) {
                System.out.println("readinFont error");
                throw new RuntimeException(e);
            }
        }
        Properties dataprop = new Properties();
        try {
            InputStreamReader reader = new InputStreamReader(new FileInputStream(fontConfig), "gbk");
            dataprop.load(reader);
            dataFilePath = dataprop.getProperty("dataFilePath");
            dataFileName = dataprop.getProperty("dataFileName");
        } catch (IOException e) {
            System.out.println("readinDataPath error");
            throw new RuntimeException(e);
        }
    }

    //****************************************************************************************************************//
    //事件相关方法
    //****************************************************************************************************************//

    //文件相关
    private void newFile() {
        //假如文件修改
        if (!textContent.equals(ta.getText())) {
            int choose = JOptionPane.showConfirmDialog(null, "是否要保存修改？");
            switch (choose) {
                case JOptionPane.YES_OPTION:
                    saveFile();
                    break;
                case JOptionPane.NO_OPTION:
                    break;
                case JOptionPane.CANCEL_OPTION:
                    return;
            }
        }

        if (!queryFileName()) return;

        try {
            File newfile = new File(filePath, fileName);
            newfile.createNewFile();
            textContent = "";
        } catch (IOException e) {
            System.out.println("newFile error");
            throw new RuntimeException(e);
        }
    }

    //选择成功返回true 反之false
    private boolean queryFileName() {
        String newfile = JOptionPane.showInputDialog(null, "输入文件名", ".txt");
        if (newfile == null || "".equals(newfile))
            return false;
        else
            fileName = newfile;
        return true;
    }

    private void openFile() {
        //假如文件修改
        if (!textContent.equals(ta.getText())) {
            int choose = JOptionPane.showConfirmDialog(null, "是否要保存修改？");
            switch (choose) {
                case JOptionPane.YES_OPTION:
                    saveFile();
                    break;
                case JOptionPane.NO_OPTION:
                    break;
                case JOptionPane.CANCEL_OPTION:
                    return;
            }
        }
        //选择文件
        JFileChooser fc = new JFileChooser(filePath);
        int ret = fc.showOpenDialog(this);
        //判断选择结果
        switch (ret) {
            case JFileChooser.APPROVE_OPTION:
                readinFile(fc);
                setTitle(title + "－" + fileName);
                break;
            case JFileChooser.ERROR_OPTION:
                JOptionPane.showMessageDialog(this, "读取出错！");
                break;
            case JFileChooser.CANCEL_OPTION:
                return;
        }

    }

    private void readinFile(JFileChooser fc) {
        StringBuilder text = new StringBuilder();
        char[] buf = new char[100000];
        int len;
        textContent = "";
        try {
            //读取文件内容  并指定内容编码为gbk
            filePath = fc.getSelectedFile().getParent() + "\\";
            fileName = fc.getSelectedFile().getName();
            InputStreamReader reader = new InputStreamReader(new FileInputStream(filePath + fileName), "gbk");
            BufferedReader in = new BufferedReader(reader, 100010);
            while ((len = in.read(buf)) != -1) {
                text.append(buf, 0, len);
                //防止text缓存移除 读进4096个字符就输出一次
                textContent += text.toString();
                text.setLength(0);
            }
            in.close();
        } catch (IOException e) {
            System.out.println("readinFile error");
            throw new RuntimeException(e);
        }
        ta.setText(textContent);
    }

    private void saveFile() {
        if (fileName == null && (ta.getText() == null || "".equals(ta.getText()))) return;

        if (fileName == null) {
            if (!queryFileName()) return;

            File newfile = new File(filePath, fileName);
            try {
                newfile.createNewFile();
            } catch (IOException e) {
                System.out.println("saveFile error");
                throw new RuntimeException(e);
            }
        }

        textContent = ta.getText();
        OutputStreamWriter writer;
        try {
            writer = new OutputStreamWriter(new FileOutputStream(fileName), "gbk");
            writer.write(textContent);
            writer.close();
        } catch (IOException e) {
            System.out.println("saveFile error");
            throw new RuntimeException(e);
        }
    }

    private void setFont() {
        setPopWindowSize(myFontDly);
        myFontDly.showFontDly();
    }

    private void setBGColor() {
        Color color = JColorChooser.showDialog(null, "选择背景颜色", Color.white);
        if (color == null)
            return;

        //getRGB的第一个字节是透明度 不需要
        int colorMask = 0x00ffffff;
        bgColor = "#" + Integer.toHexString(color.getRGB() & colorMask);

        ta.setBackground(color);
    }

    private void checkQuit() {
        //假如文件修改
        if (!textContent.equals(ta.getText())) {
            int choose = JOptionPane.showConfirmDialog(null, "是否要保存修改？");
            switch (choose) {
                case JOptionPane.YES_OPTION:
                    saveFile();
                    exit();
                    break;
                case JOptionPane.NO_OPTION:
                    exit();
                    break;
                case JOptionPane.CANCEL_OPTION:
                    return;
            }
        } else {
            int ret = JOptionPane.showConfirmDialog(this, "确定要退出系统吗？", "Java程序设计总和实验", JOptionPane.YES_NO_OPTION);
            switch (ret) {
                case JOptionPane.YES_OPTION:
                    exit();
                    break;
                case JOptionPane.NO_OPTION:
                    break;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Java上机题目

    //回文数
    private void checkHuiWen() {
        JFrame fr = new JFrame("判断回文数");
        fr.setLayout(new GridLayout(3, 1, 5, 20));

        JLabel lbl = new JLabel("请输入1-99999之间的整数：");
        JTextField textField = new JTextField(5);
        textField.setHorizontalAlignment(JTextField.CENTER);
        JButton checkBtn = new JButton("判断是否为回文数");
        JButton cancelBtn = new JButton("取消");
        JPanel panBtn = new JPanel(new BorderLayout());

        panBtn.add(checkBtn, BorderLayout.WEST);
        panBtn.add(cancelBtn, BorderLayout.EAST);

        fr.add(lbl);
        fr.add(textField);
        fr.add(panBtn);

        ActionListener listener = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (e.getSource() == cancelBtn) {
                    fr.dispose();
                    return;
                }

                String content = textField.getText();
                if (!isNumber(content) || content.length() > 5 || "".equals(content)) {
                    JOptionPane.showMessageDialog(null, "输入错误！");
                    return;
                }

                boolean ok = true;
                for (int i = 0; i < content.length() / 2; ++i) {
                    if (content.charAt(i) != content.charAt(content.length() - 1 - i))
                        ok = false;
                }
                if (ok)
                    JOptionPane.showMessageDialog(null, "这是一个回文数");
                else
                    JOptionPane.showMessageDialog(null, "这不是一个回文数");
            }
        };
        checkBtn.addActionListener(listener);
        cancelBtn.addActionListener(listener);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    //数字与英文互译
    private void transNumToEng() {
        JFrame fr = new JFrame("数字与英文互译");

        JLabel lbl = new JLabel("请输入1-99之间的整数：");
        JTextField inField = new JTextField(5);
        JTextField outField = new JTextField(5);
        JSplitPane textPanel = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, inField, outField);

        inField.setHorizontalAlignment(JTextField.CENTER);
        outField.setHorizontalAlignment(JTextField.CENTER);
        outField.setEditable(false);
        textPanel.setDividerSize(5);

        JButton checkBtn = new JButton("确定");
        JButton cancelBtn = new JButton("取消");
        JPanel panBtn = new JPanel(new BorderLayout());

        panBtn.add(checkBtn, BorderLayout.WEST);
        panBtn.add(cancelBtn, BorderLayout.EAST);

        fr.add(lbl, BorderLayout.NORTH);
        fr.add(textPanel, BorderLayout.CENTER);
        fr.add(panBtn, BorderLayout.SOUTH);

        String[] x = {"zero", "one", "two", "three", "four", "five",
                "six", "seven", "eight", "nine"};
        String[] y = {"ten", "eleven", "twelve", "thirteen", "fourteen",
                "fifteen", "sixteen", "seventeen", "eighteen",
                "nineteen"};
        String[] z = {"twenty", "thirty", "forty", "fifty",
                "sixty", "seventy", "eighty", "ninety"};
        ActionListener listener = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (e.getSource() == cancelBtn) {
                    fr.dispose();
                    return;
                }


                String s = inField.getText();
                if (isNumber(s)) {
                    int num = Integer.parseInt(s);
                    if (num < 0 || num > 100) {//判断范围
                        JOptionPane.showMessageDialog(null, "输入错误！");
                        return;
                    }

                    if (num <= 9) {
                        outField.setText(x[num]);
                    } else if (num <= 19) {
                        outField.setText(y[num % 10]);
                    } else {
                        int fst = num / 10, sec = num % 10;
                        if (sec != 0)
                            outField.setText(z[fst - 2] + " " + x[sec]);
                        else
                            outField.setText(z[fst - 2]);
                    }
                } else if (isEnglish(s, x, y, z)) {
                    String[] split = s.split(" ");
                    int len = split.length;
                    if (len == 1) {
                        for (int i = 0; i < x.length; ++i) {
                            if (x[i].equals(split[0])) {
                                outField.setText(String.valueOf(i));
                                break;
                            }
                        }
                        for (int i = 0; i < y.length; ++i) {
                            if (y[i].equals(split[0])) {
                                outField.setText("1" + i);
                                break;
                            }
                        }
                    } else if (len == 2) {
                        String fst = split[0], sec = split[1];
                        //在isNumber中已经判断sec为zero的情况，因此此处不再判断
                        String ret = "";
                        for (int i = 0; i < z.length; i++) {
                            if (z[i].equals(fst)) {
                                ret += i + 2;
                                break;
                            }
                        }
                        for (int i = 0; i < x.length; i++) {
                            if (x[i].equals(sec)) {
                                ret += i;
                                break;
                            }
                        }
                        outField.setText(ret);
                    }
                } else {
                    JOptionPane.showMessageDialog(null, "输入错误！");
                }
            }
        };
        checkBtn.addActionListener(listener);
        cancelBtn.addActionListener(listener);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        textPanel.setDividerLocation(0.5);
    }

    //统计英文数据
    private void computeWords() {
        //先获得统计数据
        int[] cnt = new int[26];
        int containsOr = 0;
        int lengthIs3 = 0;

        String content = ta.getText();
        if ("".equals(content) || content == null) {
            JOptionPane.showMessageDialog(null, "无文本数据可以统计！");
            return;
        }

        String[] split = content.split("\\W+");

        for (String s : split) {
            char first = s.charAt(0);
            if (!Character.isLetter(first)) continue;

            cnt[Character.toLowerCase(first) - 'a']++;
            if (s.contains("or"))
                containsOr++;
            if (s.length() == 3)
                lengthIs3++;
        }

        JFrame fr = new JFrame("统计英文数据");

        JSplitPane mainPanel = new JSplitPane(JSplitPane.VERTICAL_SPLIT);


        JPanel upperPanel = new JPanel(new GridLayout(2, 1, 0, 0));

        JLabel lblOr = new JLabel("含\"or\"字符串的单词书");
        JLabel lblOrNum = new JLabel(String.valueOf(containsOr));
        JPanel panOr = new JPanel();

        panOr.add(lblOr);
        panOr.add(lblOrNum);

        JLabel lblLen3 = new JLabel("长度为3的单词数");
        JLabel lblLen3Num = new JLabel(String.valueOf(lengthIs3));
        JPanel pan3 = new JPanel();

        pan3.add(lblLen3);
        pan3.add(lblLen3Num);

        upperPanel.add(panOr);
        upperPanel.add(pan3);


        JPanel lowerPanel;
        int col = 26 * 2 + 1;
        lowerPanel = new JPanel(new GridLayout(0, col, 0, 0));
        Color[] colors = new Color[]{Color.cyan.brighter(), Color.yellow.brighter(), Color.red.brighter(), Color.green.brighter()};

        int max = 0;
        for (int x : cnt) max = Math.max(max, x);
        //至少画四行
        int idx = Math.max(4, max);

        while (idx > 0) {
            lowerPanel.add(new JLabel(String.valueOf(idx)));

            for (int i = 0; i < cnt.length; i++) {
                lowerPanel.add(new JPanel());

                if (cnt[i] >= idx) {
                    JPanel tem = new JPanel();
                    tem.setBackground(colors[i % 4]);
                    lowerPanel.add(tem);
                } else {
                    lowerPanel.add(new JPanel());
                }
            }

            idx--;
        }
        lowerPanel.add(new JPanel());
        for (int i = 0; i < 26; ++i) {
            lowerPanel.add(new JLabel());
            lowerPanel.add(new JLabel(String.valueOf((char) (i + 'a'))));
        }

        mainPanel.add(upperPanel);
        mainPanel.add(lowerPanel);

        fr.add(mainPanel);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        mainPanel.setDividerLocation(0.1);
    }

    //手机号码合法性判断
    private void phoneCheck() {
        JFrame fr = new JFrame("手机号码合法性判断");
        fr.setLayout(new GridLayout(3, 1, 5, 20));

        JLabel lbl = new JLabel("请输入合法的手机号：");
        JTextField textField = new JTextField(5);
        textField.setHorizontalAlignment(JTextField.CENTER);
        JButton checkBtn = new JButton("判断合法性");
        JButton cancelBtn = new JButton("取消");
        JPanel panBtn = new JPanel(new BorderLayout());

        panBtn.add(checkBtn, BorderLayout.WEST);
        panBtn.add(cancelBtn, BorderLayout.EAST);

        fr.add(lbl);
        fr.add(textField);
        fr.add(panBtn);

        ActionListener listener = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (e.getSource() == cancelBtn) {
                    fr.dispose();
                    return;
                }

                String s = textField.getText();
                String[] split;
                if (s.startsWith("+"))
                    split = s.substring(1).split("-");
                else
                    split = s.split("-");
                //判断长度
                int len = 0;
                for (String sp : split)
                    len += sp.length();
                if (len != 13) {
                    JOptionPane.showMessageDialog(null, "长度不合法，返回1");
                    return;
                }
                //判断包含非数字
                boolean ok = true;
                for (String sp : split)
                    for (char c : sp.toCharArray())
                        if (c < '0' || c > '9') {
                            ok = false;
                            break;
                        }
                if (!ok) {
                    JOptionPane.showMessageDialog(null, "包含非数字，返回2");
                    return;
                }
                //不是86打头
                if (!("86".equals(s.substring(0, 2)) || "+86".equals(s.substring(0, 3)))) {
                    JOptionPane.showMessageDialog(null, "不是86打头，返回3");
                    return;
                }
                //格式有问题
                //8613957177889这种只有长度限制 上面已判断
                //+8613957177889同理
                //86-13957177889 或 +86-13957177889
                ok = true;
                if (!(s.length() == 13 || s.length() == 14 || s.length() == 15 || s.length() == 17))
                    ok = false;
                if (split.length == 2)
                    if (!(split[0].length() == 2 && split[1].length() == 11))
                        ok = false;
                if (split.length == 4)
                    if (!(split[0].length() == 2 && split[1].length() == 3 && split[2].length() == 4 && split[3].length() == 4))
                        ok = false;
                if (!ok) {
                    JOptionPane.showMessageDialog(null, "格式错误， 返回4");
                    return;
                }
                //合法
                JOptionPane.showMessageDialog(null, "号码合法，返回0");
            }
        };
        checkBtn.addActionListener(listener);
        cancelBtn.addActionListener(listener);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    private void textSum() {
        JFrame fr = new JFrame("文本文件求和");

        JLabel lbl = new JLabel();

        JProgressBar bar = new JProgressBar(0, 100);
        bar.setStringPainted(true);
        bar.setBackground(Color.white);
        bar.setForeground(Color.blue);

        JButton btn = new JButton("开始");
        JPanel pan = new JPanel(new FlowLayout(FlowLayout.RIGHT));
        pan.add(btn);

        fr.add(lbl, BorderLayout.NORTH);
        fr.add(bar, BorderLayout.CENTER);
        fr.add(pan, BorderLayout.SOUTH);


        BarThread stepper = new BarThread(bar, lbl);
        ActionListener listener = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if ("开始".equals(btn.getText())) {
                    stepper.setContent(ta.getText());
                    stepper.start();
                    btn.setText("取消");
                    return;
                }
                if ("取消".equals(btn.getText())) {
                    stepper.stop(true);
                    fr.dispose();
                }
            }
        };

        btn.addActionListener(listener);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //通讯录
    //通讯录维护
    private void displayData() throws Exception {
        JFrame fr = new JFrame("通讯录");

        Object[] colName = {"序号", "姓名", "性别", "移动电话号码", "Email", "QQ号"};
        //读取数据
        Object[][] rowData;
        File file = new File(dataFilePath + dataFileName);
        if (file.exists()) {
            ObjectInputStream in = new ObjectInputStream(new FileInputStream(file));
            mana = (ContactManager) (in.readObject());
        } else {
            mana = new ContactManager();
        }
        List<Contact> all = mana.getAllContacts();
        rowData = new Object[all.size()][6];
        for (int i = 0; i < all.size(); ++i) {
            Contact c = all.get(i);
            rowData[i][0] = c.getnId();
            rowData[i][1] = c.getsName();
            rowData[i][2] = c.getByteSex() == 1 ? "男" : "女";
            rowData[i][3] = c.getsCellPhone();
            rowData[i][4] = c.getsEmail();
            rowData[i][5] = c.getsInstantMessager();
        }

        JTable table = new JTable(rowData, colName);
        table.setRowSelectionAllowed(true);
        JScrollPane tablePan = new JScrollPane(table);

        JButton addBtn = new JButton("添加");
        JButton delBtn = new JButton("删除");
        JButton chaBtn = new JButton("修改");
        JButton queBtn = new JButton("查询");
        JButton extBtn = new JButton("关闭");
        JPanel btnPan = new JPanel();

        btnPan.add(addBtn);
        btnPan.add(delBtn);
        btnPan.add(chaBtn);
        btnPan.add(queBtn);
        btnPan.add(extBtn);

        ActionListener listener = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (e.getSource() == addBtn) {
                    addContact(table);
                }
                if (e.getSource() == delBtn) {
                    int row = table.getSelectedRow();
                    mana.removeContact((Integer) table.getValueAt(row, 0));
                    flushTable(table);
                }
                if (e.getSource() == chaBtn) {
                    changeContact(table.getSelectedRow(), table);
                }
                if (e.getSource() == queBtn) {
                    queryContact(table);
                }
                if (e.getSource() == extBtn) {
                    try {
                        saveData();
                    } catch (Exception ex) {
                        System.out.println("saveData error");
                        throw new RuntimeException(ex);
                    }
                    fr.dispose();
                }
            }
        };

        fr.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                try {
                    saveData();
                } catch (Exception ex) {
                    System.out.println("windowClosing error");
                    throw new RuntimeException(ex);
                }
                fr.dispose();
            }
        });

       addBtn.addActionListener(listener);
       delBtn.addActionListener(listener);
       chaBtn.addActionListener(listener);
       queBtn.addActionListener(listener);
       extBtn.addActionListener(listener);

        fr.add(tablePan);
        fr.add(btnPan, BorderLayout.SOUTH);
        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DO_NOTHING_ON_CLOSE);
    }

    private void addContact(JTable table) {
        JFrame fr = new JFrame("添加联系人");

        JLabel lbl1 = new JLabel("姓名");
        JTextField nameField = new JTextField(10);
        JPanel pan1 = new JPanel();
        pan1.add(lbl1);
        pan1.add(nameField);

        JLabel lbl2 = new JLabel("性别");
        JComboBox<String> sexCom = new JComboBox<>();
        sexCom.addItem("男");
        sexCom.addItem("女");
        sexCom.setSelectedItem("男");
        JPanel pan2 = new JPanel();
        pan2.add(lbl2);
        pan2.add(sexCom);

        JLabel lbl3 = new JLabel("移动电话号码");
        JTextField phoneFiled = new JTextField(10);
        JPanel pan3 = new JPanel();
        pan3.add(lbl3);
        pan3.add(phoneFiled);

        JLabel lbl4 = new JLabel("Email");
        JTextField emailField = new JTextField(10);
        JPanel pan4 = new JPanel();
        pan4.add(lbl4);
        pan4.add(emailField);

        JLabel lbl5 = new JLabel("qq");
        JTextField qqField = new JTextField(10);
        JPanel pan5 = new JPanel();
        pan5.add(lbl5);
        pan5.add(qqField);

        JButton yes = new JButton("确定");
        JButton no = new JButton("取消");
        JPanel pan6 = new JPanel();
        pan6.add(yes);
        pan6.add(no);

        yes.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String name = nameField.getText();
                String sex = (String) (sexCom.getSelectedItem());
                String phone = phoneFiled.getText();
                String email = emailField.getText();
                String qq = qqField.getText();
                if (name.isEmpty() || phone.isEmpty() || email.isEmpty()|| qq.isEmpty()) {
                    JOptionPane.showMessageDialog(null, "信息不完整！");
                    return;
                }
                byte bytesex = (byte) ("男".equals(sex) ? 1 : 0);
                mana.addContact(new Contact(name, bytesex, phone, email, qq));
                fr.dispose();
                flushTable(table);
            }
        });
        no.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                fr.dispose();
            }
        });

        JPanel mainPan = new JPanel();
        mainPan.setLayout(new BoxLayout(mainPan, BoxLayout.Y_AXIS));
        mainPan.add(pan1);
        mainPan.add(pan2);
        mainPan.add(pan3);
        mainPan.add(pan4);
        mainPan.add(pan5);
        mainPan.add(pan6);
        fr.add(mainPan);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    private void changeContact(int oldid, JTable table) {
        JFrame fr = new JFrame("修改联系人");

        JLabel lbl1 = new JLabel("姓名");
        JTextField nameField = new JTextField(10);
        JPanel pan1 = new JPanel();
        pan1.add(lbl1);
        pan1.add(nameField);

        JLabel lbl2 = new JLabel("性别");
        JComboBox<String> sexCom = new JComboBox<>();
        sexCom.addItem("男");
        sexCom.addItem("女");
        sexCom.setSelectedItem("男");
        JPanel pan2 = new JPanel();
        pan2.add(lbl2);
        pan2.add(sexCom);

        JLabel lbl3 = new JLabel("移动电话号码");
        JTextField phoneFiled = new JTextField(10);
        JPanel pan3 = new JPanel();
        pan3.add(lbl3);
        pan3.add(phoneFiled);

        JLabel lbl4 = new JLabel("Email");
        JTextField emailField = new JTextField(10);
        JPanel pan4 = new JPanel();
        pan4.add(lbl4);
        pan4.add(emailField);

        JLabel lbl5 = new JLabel("qq");
        JTextField qqField = new JTextField(10);
        JPanel pan5 = new JPanel();
        pan5.add(lbl5);
        pan5.add(qqField);

        JButton yes = new JButton("修改");
        JButton no = new JButton("取消");
        JPanel pan6 = new JPanel();
        pan6.add(yes);
        pan6.add(no);

        yes.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String name = nameField.getText();
                String sex = (String) (sexCom.getSelectedItem());
                String phone = phoneFiled.getText();
                String email = emailField.getText();
                String qq = qqField.getText();
                if (name.isEmpty() || phone.isEmpty() || email.isEmpty()|| qq.isEmpty()) {
                    JOptionPane.showMessageDialog(null, "信息不完整！");
                    return;
                }
                byte bytesex = (byte) ("男".equals(sex) ? 1 : 0);
                mana.updateContact(oldid, new Contact(name, bytesex, phone, email, qq));
                fr.dispose();
                flushTable(table);
            }
        });
        no.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                fr.dispose();
            }
        });

        JPanel mainPan = new JPanel();
        mainPan.setLayout(new BoxLayout(mainPan, BoxLayout.Y_AXIS));
        mainPan.add(pan1);
        mainPan.add(pan2);
        mainPan.add(pan3);
        mainPan.add(pan4);
        mainPan.add(pan5);
        mainPan.add(pan6);
        fr.add(mainPan);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    private void queryContact(JTable table) {
        JFrame fr = new JFrame("查询联系人");

        JLabel lbl = new JLabel("输入关键字：");
        JTextField txtField = new JTextField(10);
        JButton queBtn = new JButton("查询");


        queBtn.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                JFrame fr = new JFrame("查询结果");
                Object[] colName = {"序号", "姓名", "性别", "移动电话号码", "Email", "QQ号"};

                List<Contact> all = mana.searchContacts(txtField.getText());
                Object[][] rowData = new Object[all.size()][6];
                for (int i = 0; i < all.size(); ++i) {
                    Contact c = all.get(i);
                    rowData[i][0] = c.getnId();
                    rowData[i][1] = c.getsName();
                    rowData[i][2] = c.getByteSex();
                    rowData[i][3] = c.getsCellPhone();
                    rowData[i][4] = c.getsEmail();
                    rowData[i][5] = c.getsInstantMessager();
                }
                JTable table = new JTable(rowData, colName);
                fr.add(table);

                setPopWindowSize(fr);
                fr.setVisible(true);
                fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
            }
        });



        fr.add(lbl, BorderLayout.NORTH);
        fr.add(txtField, BorderLayout.CENTER);
        fr.add(queBtn, BorderLayout.SOUTH);

        setPopWindowSize(fr);
        fr.setVisible(true);
        fr.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
    }

    private void flushTable(JTable table) {
        Object[] colName = {"序号", "姓名", "性别", "移动电话号码", "Email", "QQ号"};

        List<Contact> all = mana.getAllContacts();
        Object[][] rowData = new Object[all.size()][6];
        for (int i = 0; i < all.size(); ++i) {
            Contact c = all.get(i);
            rowData[i][0] = c.getnId();
            rowData[i][1] = c.getsName();
            rowData[i][2] = c.getByteSex() == 1 ? "男" : "女";
            rowData[i][3] = c.getsCellPhone();
            rowData[i][4] = c.getsEmail();
            rowData[i][5] = c.getsInstantMessager();
        }

        table.setModel(new DefaultTableModel(rowData, colName));
    }

    private void saveData() throws Exception {
        ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(dataFilePath + dataFileName));
        out.writeObject(mana);
        out.flush();
        out.close();
    }

    //通讯录储存文件设置
    private void setDataPath() {
        JFileChooser jf = new JFileChooser(filePath);
        jf.setSelectedFile(new File(dataFilePath, dataFileName));
        jf.showSaveDialog(null);
        dataFilePath = jf.getSelectedFile().getParent().replace("\\", "\\\\");
        dataFileName = jf.getSelectedFile().getName();
    }

    //****************************************************************************************************************//
    //通用方法
    //****************************************************************************************************************//
    private void exit() {
        //保存字体 字体大小 字体颜色 背景颜色
        saveFont();
        //保存数据文件路径
        saveDataPath();
        //保存结束后退出
        myFontDly.dispose();
        dispose();
    }

    private void saveFont() {
        File fontConfig = new File("font.properties");
        String s = "";
        if (!fontConfig.exists()) {
            try {
                fontConfig.createNewFile();
            } catch (IOException e) {
                System.out.println("saveFont error");
                throw new RuntimeException(e);
            }
        }
        try {
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("font.properties"), "gbk"));
            s = String.format("font=%s\nfontSize=%d\nfontColor=%s\nbgColor=%s", font, fontSize, fontColor, bgColor);

            writer.write(s);
            writer.flush();
            writer.close();
        } catch (IOException e) {
            System.out.println("saveFont error");
            throw new RuntimeException(e);
        }
    }

    private void saveDataPath() {
        File fontConfig = new File("datapath.properties");
        String s = "";
        if (!fontConfig.exists()) {
            try {
                fontConfig.createNewFile();
            } catch (IOException e) {
                System.out.println("saveFont error");
                throw new RuntimeException(e);
            }
        }
        try {
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("datapath.properties"), "gbk"));
            s = String.format("dataFilePath=%s\ndataFileName=%s", dataFilePath.replace("\\", "\\\\"), dataFileName);

            writer.write(s);
            writer.flush();
            writer.close();
        } catch (IOException e) {
            System.out.println("saveDataPath error");
            throw new RuntimeException(e);
        }
    }

    //****************************************************************************************************************//
    //主程入口
    //****************************************************************************************************************//
    public static void main(String[] args) {
        J202203150619_4 fr = new J202203150619_4(title);

        //将右上角的差换为自己的退出函数
        fr.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        fr.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                fr.checkQuit();
            }
        });
        centerWindow(fr);
        fr.setVisible(true);
    }

    //****************************************************************************************************************//
    //辅助方法
    //****************************************************************************************************************//
    public static void centerWindow(Window f) {
        //获得桌面大小
        Toolkit tk = f.getToolkit();
        Dimension dm = tk.getScreenSize();
        //让窗口居中显示
        f.setLocation((int) (dm.getWidth() - f.getWidth()) / 2, (int) (dm.getHeight() - f.getHeight()) / 2);
    }

    public static void setPopWindowSize(Window f) {
        //获得桌面大小
        Toolkit tk = f.getToolkit();
        Dimension dm = tk.getScreenSize();

        f.setSize((int) (dm.getWidth() * 0.25), (int) (dm.getHeight() * 0.25));
        centerWindow(f);
    }

    public static Color stringToColor(String s) {
        s = s.toLowerCase();
        Color ret = null;
        switch (s) {
            case "white":
                ret = Color.white;
                break;
            case "black":
                ret = Color.black;
                break;
            case "gray":
                ret = Color.gray;
                break;
            case "red":
                ret = Color.red;
                break;
            case "pink":
                ret = Color.pink;
                break;
            case "orange":
                ret = Color.orange;
                break;
            case "yellow":
                ret = Color.yellow;
                break;
            case "green":
                ret = Color.green;
                break;
            case "cyan":
                ret = Color.cyan;
                break;
            case "blue":
                ret = Color.blue;
                break;
            default:
                ret = Color.black;
        }
        return ret;
    }

    public boolean isNumber(String s) {
        for (byte c : s.getBytes())
            if (c < '0' || c > '9')
                return false;
        return true;
    }

    public static boolean isEnglish(String s, String[] x, String[] y, String[] z) {
        String[] split = s.split(" ");
        int len = split.length;
        if (len == 1) {
            return inArray(s, x) || inArray(s, y);
        } else if (len == 2) {
            String fst = split[0], sec = split[1];
            if ("zero".equals(sec)) { // 防止 thirty zero 这样的案例通过
                return false;
            }
            return inArray(fst, z) && inArray(sec, x);
        }
        return false;
    }

    private static boolean inArray(String s, String[] arr) {
        for (String num : arr) {
            if (num.equals(s)) {
                return true;
            }
        }
        return false;
    }

    //****************************************************************************************************************//
    //自定义内部类
    //****************************************************************************************************************//

    //有助记符构造函数的按钮
    static class MyMenu extends JMenu {
        private MyMenu(String label) {
            super(label);
        }

        private MyMenu(String label, int mnemonic) {
            super(label);
            setMnemonic(mnemonic);
        }
    }

    //选字体的界面框
    class MyFontDly extends JFrame {
        //两个按钮
        JButton btnOk = new JButton("确定");
        JButton btnCancel = new JButton("取消");
        JPanel panButtons = new JPanel();

        //字体名称
        String _font;
        JLabel lblFont = new JLabel("字体名称：", JLabel.LEFT);
        JComboBox<String> cbFontFamily = new JComboBox<>();
        JPanel panFont = new JPanel();

        //字体大小
        int _fontSize;
        JLabel lblFontSize = new JLabel("字体大小：", JLabel.LEFT);
        JComboBox<String> cbFontSize = new JComboBox<>();
        JPanel panFontSize = new JPanel();

        //字体颜色
        String _fontColor;
        JLabel lblFontColor = new JLabel("字体颜色：", JLabel.LEFT);
        JComboBox<String> cbFontColor = new JComboBox<>();
        JPanel panFontColor = new JPanel();
        final String[] fontColors = {"white", "black", "gray", "red", "pink", "orange", "yellow", "cyan", "blue"};

        //字体预览
        JLabel lblFontPreview = new JLabel("字体预览：", JLabel.LEFT);
        JLabel lblTextPreview = new JLabel("java 你好我好大家好", JLabel.CENTER);
        JPanel panFontPreView = new JPanel();

        public void showFontDly() {
            setVisible(true);
        }

        public void hideFontDly() {
            setVisible(false);
        }

        private MyFontDly(String title) {
            super(title);

            GridLayout gl = new GridLayout(5, 1, 5, 5);
            setLayout(gl);

            //初始化字体框
            InitFonts();

            //字体名称
            _font = font;
            panFont.setLayout(new BorderLayout());
            panFont.add(lblFont, BorderLayout.WEST);
            panFont.add(cbFontFamily, BorderLayout.CENTER);

            cbFontFamily.addItemListener(new ItemListener() {
                @Override
                public void itemStateChanged(ItemEvent e) {
                    fontNameItemStateChanged(e);
                }
            });
            add(panFont);

            //字体大小
            _fontSize = fontSize;
            panFontSize.setLayout(new BorderLayout());
            panFontSize.add(lblFontSize, BorderLayout.WEST);
            panFontSize.add(cbFontSize, BorderLayout.CENTER);

            cbFontSize.addItemListener(new ItemListener() {
                @Override
                public void itemStateChanged(ItemEvent e) {
                    fontSizeItemStateChanged(e);
                }
            });
            add(panFontSize);

            //字体颜色
            _fontColor = fontColor;
            panFontColor.setLayout(new BorderLayout());
            panFontColor.add(lblFontColor, BorderLayout.WEST);
            panFontColor.add(cbFontColor, BorderLayout.CENTER);

            cbFontColor.addItemListener(new ItemListener() {
                @Override
                public void itemStateChanged(ItemEvent e) {
                    fontColorItemStateChanged(e);
                }
            });
            add(panFontColor);

            //字体预览
            panFontPreView.setLayout(new BorderLayout());
            panFontPreView.add(lblFontPreview, BorderLayout.WEST);
            panFontPreView.add(lblTextPreview, BorderLayout.CENTER);
            lblTextPreview.setBorder(BorderFactory.createEtchedBorder());
            lblTextPreview.setOpaque(true);
            add(panFontPreView);

            //按钮设置
            panButtons.setLayout(new FlowLayout(FlowLayout.RIGHT));
            panButtons.add(btnOk);
            panButtons.add(btnCancel);
            add(panButtons);

            //初始化下拉框
            cbFontFamily.setSelectedItem(_font);
            cbFontSize.setSelectedItem(Integer.toString(_fontSize));
            cbFontColor.setSelectedItem(_fontColor);

            //注册事件
            btnOk.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    btnActionPerformed(e);
                }
            });
            btnCancel.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    btnActionPerformed(e);
                }
            });
            //初始化字体预览
            lblTextPreview.setFont(new Font(_font, Font.PLAIN, _fontSize));
            lblTextPreview.setForeground(stringToColor(_fontColor));
        }

        private void btnActionPerformed(ActionEvent e) {
            if (e.getSource() == btnOk) {
                font = _font;
                fontSize = _fontSize;
                fontColor = _fontColor;
                ta.setFont(new Font(font, Font.PLAIN, fontSize));
                ta.setForeground(stringToColor(fontColor));
            }
            if (e.getSource() == btnCancel) {
                _font = font;
                _fontSize = fontSize;
                _fontColor = fontColor;
            }

            //隐藏窗口
            hideFontDly();
        }

        public void fontNameItemStateChanged(ItemEvent e) {
            _font = cbFontFamily.getSelectedItem().toString();
            lblTextPreview.setFont(new Font(_font, Font.PLAIN, _fontSize));
            lblTextPreview.setForeground(stringToColor(_fontColor));
        }

        public void fontSizeItemStateChanged(ItemEvent e) {
            _fontSize = Integer.parseInt(cbFontSize.getSelectedItem().toString());
            lblTextPreview.setFont(new Font(_font, Font.PLAIN, _fontSize));
            lblTextPreview.setForeground(stringToColor(_fontColor));
        }

        public void fontColorItemStateChanged(ItemEvent e) {
            _fontColor = cbFontColor.getSelectedItem().toString();
            lblTextPreview.setFont(new Font(_font, Font.PLAIN, _fontSize));
            lblTextPreview.setForeground(stringToColor(_fontColor));
        }

        public void InitFonts() {
            GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
            String[] fontList = ge.getAvailableFontFamilyNames();
            for (int i = fontList.length - 1; i >= 0; --i)
                cbFontFamily.addItem(fontList[i]);

            for (int i = 9; i <= 72; ++i)
                cbFontSize.addItem(Integer.toString(i));

            for (int i = 0; i < fontColors.length; ++i)
                cbFontColor.addItem(fontColors[i]);
        }
    }

    //带线程的进度条
    class BarThread extends Thread {
        String content;
        JLabel lbl;
        JProgressBar progressBar;
        boolean m_bStopped;

        //构造方法
        public BarThread(JProgressBar bar, JLabel lbl) {
            progressBar = bar;
            this.lbl = lbl;
            m_bStopped = false;
        }

        //线程体
        public void run() {
            String[] split = content.split("\\r\\n|\\n|\\r");
            long size = split.length;
            double sum = 0;

            long startTime = System.currentTimeMillis();
            for (int i = 0; i < size; ++i) {
                if (m_bStopped) break;

                String s = split[i];
                int idx = s.indexOf('=');

                sum += Double.parseDouble(s.substring(idx + 1));

                double nowPercent = 1.0 * i / size;
                long passTime = System.currentTimeMillis() - startTime;
                long needTime = passTime * size / (i + 1) - passTime;

                lbl.setText(String.format("当前和为%f,正在计算变量%s(%d/%d),大约还剩下%d秒", sum, s.substring(0, idx), i + 1, size, needTime / 1000));
                progressBar.setValue((int) Math.ceil(nowPercent * 100));
            }
        }

        public void setContent(String content) {
            this.content = content;
        }

        //设置停止
        public void stop(boolean bStopped) {
            m_bStopped = bStopped;
        }
    }
}

//联系人管理类 方便管理
class ContactManager implements Serializable{
    static final long serialVersionUID = 1L;

    private List<Contact> contacts;
    private int idCnt;

    public ContactManager() {
        contacts = new ArrayList<>();
        idCnt = 0;
    }

    public void addContact(Contact contact) {
        contacts.add(contact);
        idCnt = contacts.size();
        contact.setnId(idCnt);
    }

    public void removeContact(int id) {
        for (Contact c : contacts) {
            if (c.getnId() == id) {
                contacts.remove(c);
                return;
            }
        }
    }
    public void removeContact(Contact contact) {
        contacts.remove(contact);
    }

    public void updateContact(int oldid, Contact _new) {
        Contact _old = contacts.get(oldid);
        _old.setsName(_new.getsName());
        _old.setByteSex(_new.getByteSex());
        _old.setsCellPhone(_new.getsCellPhone());
        _old.setsEmail(_new.getsEmail());
        _old.setsInstantMessager(_new.getsInstantMessager());
    }

    public void updateContact(Contact _old, Contact _new) {
        contacts.remove(_old);
        contacts.add(_new);
    }

    public List<Contact> searchContacts(String keyword) {
        List<Contact> results = new ArrayList<>();
        for (Contact contact : contacts) {
            // 根据需要进行模糊查询
            // 判断姓名、工作单位、备注字段是否包含 keyword
            if (contact.getsName().contains(keyword) ||
                    contact.getsCellPhone().contains(keyword) ||
                    contact.getsEmail().contains(keyword) ||
                    (String.valueOf(contact.getnId())).contains(keyword)) {
                results.add(contact);
            }
        }
        return results;
    }

    public List<Contact> getAllContacts() {
        return contacts;
    }
}

//联系人类
class Contact implements Serializable {
    static final long serialVersionUID = 1L;
    int nId;
    String sName; //非空
    byte byteSex; //1表示男，0表示女，非空
    String sAddress;
    String sCompany;
    String sPostalCode;
    String sHomeTele;
    String sOfficeTele;
    String sFax;
    String sCellPhone;
    String sEmail;
    String sInstantMessager;
    Date dateBirthday;
    String sMemo;

    public Contact() {
    }

    public Contact(String sName, byte byteSex, String sCellPhone, String sEmail, String sInstantMessager) {
        this.sName = sName;
        this.byteSex = byteSex;
        this.sCellPhone = sCellPhone;
        this.sEmail = sEmail;
        this.sInstantMessager = sInstantMessager;
    }

    public Contact(String sName, byte byteSex, String sAddress, String sCompany, String sPostalCode, String sHomeTele, String sOfficeTele, String sFax, String sCellPhone, String sEmail, String sInstantMessager, Date dateBirthday, String sMemo) {
        this.sName = sName;
        this.byteSex = byteSex;
        this.sAddress = sAddress;
        this.sCompany = sCompany;
        this.sPostalCode = sPostalCode;
        this.sHomeTele = sHomeTele;
        this.sOfficeTele = sOfficeTele;
        this.sFax = sFax;
        this.sCellPhone = sCellPhone;
        this.sEmail = sEmail;
        this.sInstantMessager = sInstantMessager;
        this.dateBirthday = dateBirthday;
        this.sMemo = sMemo;
    }

    public int getnId() {
        return nId;
    }

    public void setnId(int nId) {
        this.nId = nId;
    }

    public String getsName() {
        return sName;
    }

    public void setsName(String sName) {
        this.sName = sName;
    }

    public byte getByteSex() {
        return byteSex;
    }

    public void setByteSex(byte byteSex) {
        this.byteSex = byteSex;
    }

    public String getsAddress() {
        return sAddress;
    }

    public void setsAddress(String sAddress) {
        this.sAddress = sAddress;
    }

    public String getsCompany() {
        return sCompany;
    }

    public void setsCompany(String sCompany) {
        this.sCompany = sCompany;
    }

    public String getsPostalCode() {
        return sPostalCode;
    }

    public void setsPostalCode(String sPostalCode) {
        this.sPostalCode = sPostalCode;
    }

    public String getsHomeTele() {
        return sHomeTele;
    }

    public void setsHomeTele(String sHomeTele) {
        this.sHomeTele = sHomeTele;
    }

    public String getsOfficeTele() {
        return sOfficeTele;
    }

    public void setsOfficeTele(String sOfficeTele) {
        this.sOfficeTele = sOfficeTele;
    }

    public String getsFax() {
        return sFax;
    }

    public void setsFax(String sFax) {
        this.sFax = sFax;
    }

    public String getsCellPhone() {
        return sCellPhone;
    }

    public void setsCellPhone(String sCellPhone) {
        this.sCellPhone = sCellPhone;
    }

    public String getsEmail() {
        return sEmail;
    }

    public void setsEmail(String sEmail) {
        this.sEmail = sEmail;
    }

    public String getsInstantMessager() {
        return sInstantMessager;
    }

    public void setsInstantMessager(String sInstantMessager) {
        this.sInstantMessager = sInstantMessager;
    }

    public Date getDateBirthday() {
        return dateBirthday;
    }

    public void setDateBirthday(Date dateBirthday) {
        this.dateBirthday = dateBirthday;
    }

    public String getsMemo() {
        return sMemo;
    }

    public void setsMemo(String sMemo) {
        this.sMemo = sMemo;
    }
}
