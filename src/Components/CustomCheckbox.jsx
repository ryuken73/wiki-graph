import React from 'react';
import Checkbox from '@mui/material/Checkbox';

const SmallCheckBox = props => {
  const {id, checked, setChecked, handleClick=()=>{}} = props;
  const handleChange = React.useCallback(event => {
    setChecked(event.target.checked, id);
  }, [setChecked])
  return (
    <div>
      <Checkbox
        {...props}
        checked={checked}
        onChange={handleChange}
        onClick={handleClick}
        sx={{ 
            color: '#6c5f5f',
            padding: '0px',
            '&.Mui-checked': {
              color: 'white',
              opacity: 0.5
            },
            '&:hover': {
              opacity: 0.8
            },
            '.MuiSvgIcon-root': { fontSize: 20 } 
        }}
      />
    </div>
  );
}

export default React.memo(SmallCheckBox);