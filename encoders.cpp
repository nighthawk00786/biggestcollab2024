
// odometry functions
void update_encoder1()
{
  if (digitalRead(ENCB[0]) == 1)
  {
    enc_count[0]++;
  }
  else
  {
    enc_count[0]--;
  }
}

void update_encoder2()
{
  if (digitalRead(ENCB[1]) == 1)
  {
    enc_count[1]--;
  }
  else
  {
    enc_count[1]++;
  }
}

void update_encoder3()
{
  if (digitalRead(ENCB[2]) == 1)
  {
    enc_count[2]++;
  }
  else
  {
    enc_count[2]--;
  }
}