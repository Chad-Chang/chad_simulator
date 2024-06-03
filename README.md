# RW_Controller를 rw_posPID의 입력변수 바꾸고 싶다.
- before
  - rw_posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx,int Leg_num)
- after
  - rw_posPID(Vector2d rw_target,Vector2d current_rw , int idx,int Leg_num)
    
